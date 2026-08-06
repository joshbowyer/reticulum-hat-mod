##############################################################################
# SX126xInterface.py                                                         #
#                                                                            #
# A Reticulum custom interface that drives SX1262/SX1268 LoRa chips         #
# directly over SPI on Linux SBCs (Raspberry Pi, Orange Pi, femtofox,       #
# etc). No separate RNode MCU required.                                      #
#                                                                            #
# Pin/GPIO resolution model:                                                 #
#   Two-stage profile lookup so SBC and radio board can evolve independently.#
#                                                                            #
#     Stage 1 — Platform profile (`platform = <name>`):                      #
#       Describes the SBC: which gpiochip device, default SPI bus, and      #
#       a `header_pin_to_line` table that maps *physical 40-pin-header* pin  #
#       numbers to `(gpiochip, line_offset)` tuples for THIS specific SBC.   #
#       The table is populated on-demand as new HATs add new pins; only the  #
#       pins actually used by known HATs need entries.                      #
#                                                                            #
#     Stage 2 — Radio board / HAT profile (`radio_board = <name>`):          #
#       Describes the radio board's wiring in terms of physical 40-pin      #
#       header pin numbers (portable across SBCs with the same header        #
#       layout). Also carries spi_bus/spi_cs and radio-electronics fields    #
#       (TCXO, RF switch, etc.).                                             #
#                                                                            #
#   Resolution: for each pin the board specifies, the platform's             #
#   `header_pin_to_line` is consulted to get `(gpiochip, line_offset)`.     #
#   Unknown platform/board/unsupported-pin combos raise clear ValueErrors.   #
#                                                                            #
#   Escape hatch — `radio_board = custom`:                                   #
#     Bypass the profile system entirely. The user specifies `gpiochip` and #
#     `pin_irq`/`pin_busy`/etc. directly in the interface config block,     #
#     interpreted as gpiochip line offsets (NOT physical pins).              #
#                                                                            #
#   Per-key overrides in the interface config block still win over profile-  #
#   derived values; a WARNING is logged when this happens.                  #
#                                                                            #
#   Backward compatibility: if neither `platform` nor `radio_board` is set   #
#   in the interface config block, the legacy direct-gpiochip-line behaviour #
#   is used (pin_irq etc. = BCM gpiochip line offsets). New configs should   #
#   always opt in by setting at least one of the two profile keys.           #
#                                                                            #
# Architecture:                                                              #
#   - One thread owns ALL radio state transitions (radio-owner thread).     #
#     It is the only thread that ever touches the radio.                    #
#   - process_outgoing() (called by Reticulum's Transport thread) is a      #
#     thin queue producer: it pushes the raw bytes onto a bounded queue    #
#     and returns immediately. No blocking, no CSMA, no TX.                 #
#   - The radio-owner thread loop:                                           #
#         1) block on the SX126x IRQ edge-wait (short timeout)              #
#         2) if IRQ fired, dispatch it (RX done / TX done / etc.)           #
#         3) drain one TX-queue entry if the airtime budget allows and      #
#            CAD says the channel is clear                                  #
#         4) loop                                                            #
#   - Split-frame framing is preserved verbatim (verified RNode-compatible).#
#   - CAD-based carrier sense replaces the old stale-RSSI check.            #
#   - Sliding-window airtime accounting via deques (no spike-after-reset     #
#     artifact). Packets are dropped when they would exceed the budget,      #
#     not silently queued behind a never-drained packet_queue.              #
#   - SPI/lockup recovery: count consecutive SPI failures, escalate to       #
#     full radio reset + reinit, mark interface offline if reinit itself    #
#     fails repeatedly.                                                      #
#                                                                            #
# Bundled profiles (see PLATFORM_PROFILES and BOARD_PROFILES dicts below):   #
#   platforms: raspberry-pi, luckfox-pico                                    #
#   boards:    meshadv-pi-hat-v1.1, femtofox-integrated-v1, station-g3,      #
#              generic-sx1262-manual                                        #
#                                                                            #
# Place SX126xInterface.py and vendored_sx126x.py in                         #
# ~/.reticulum/interfaces/ and add an interface config block to             #
# ~/.reticulum/config                                                        #
#                                                                            #
# Example config (profile-based, recommended):                              #
#                                                                            #
#   [[MeshAdv LoRa]]                                                         #
#     type = SX126xInterface                                                 #
#     interface_enabled = True                                                #
#     frequency = 915000000                                                  #
#     bandwidth = 125000                                                     #
#     spreadingfactor = 8                                                    #
#     codingrate = 5                                                         #
#     txpower = 22                                                           #
#     platform = raspberry-pi                                                #
#     radio_board = meshadv-pi-hat-v1.1                                      #
#     dio3_tcxo_voltage = 1.8                                                #
#                                                                            #
# Example config (escape hatch):                                              #
#                                                                            #
#   [[My Hand-Wired Board]]                                                  #
#     type = SX126xInterface                                                 #
#     interface_enabled = True                                                #
#     frequency = 915000000                                                  #
#     txpower = 17                                                           #
#     platform = luckfox-pico                                                #
#     radio_board = custom                                                   #
#     gpiochip = gpiochip1                                                   #
#     pin_irq = 23                                                           #
#     pin_busy = 22                                                          #
#     pin_reset = 25                                                         #
#     pin_txen = -1                                                          #
#     pin_rxen = 24                                                          #
#     spi_bus = 0                                                            #
#     spi_cs = 0                                                             #
#                                                                            #
# License: MIT (matches the rest of this project)                            #
##############################################################################

# Custom interfaces loaded by Reticulum via exec() have "Interface" and "RNS"
# injected into their globals. We import additional stdlib modules we need
# here. Do NOT import vendored_sx126x at module top — see _load_vendored_driver
# for the explanation of why it's loaded inside __init__.
import os
import sys
import copy
import threading
import time
import math
import random
import queue
import json
import importlib.util
from collections import deque


def _rnode_preamble_symbols(sf, bandwidth, cr):
    """Replicate mainline RNode_Firmware's dynamic preamble-length auto-tune
    (updateBitrate() in Utilities.h) so this driver's default preamble
    matches what a stock RNode transmits/expects for a given SF/BW/CR,
    instead of an arbitrary fixed value.

    RNode targets ~24ms of preamble airtime (less on fast links), with a
    hard floor of 18 symbols. See the long comment at the preamble_length
    config-parsing site for the full rationale.
    """
    try:
        symbol_time_ms = (2.0 ** sf / bandwidth) * 1000.0
        bitrate = sf * ((4.0 / cr) / (2.0 ** sf / (bandwidth / 1000.0))) * 1000.0

        target_ms = 24.0  # LORA_PREAMBLE_TARGET_MS
        if bitrate > 30000:  # LORA_FAST_THRESHOLD_BPS
            target_ms -= 18.0  # LORA_PREAMBLE_FAST_DELTA

        target_symbols = target_ms / symbol_time_ms
        if target_symbols < 18:  # LORA_PREAMBLE_SYMBOLS_MIN
            target_symbols = 18
        else:
            target_symbols = math.ceil(target_symbols)

        return int(target_symbols)
    except Exception:
        return 18  # safe RNode-compatible floor if anything above misbehaves


def _load_vendored_driver():
    """Locate and load vendored_sx126x.py alongside this interface module.

    Reticulum loads custom interfaces via exec(), with __file__ NOT injected
    into the exec globals. So we can't rely on __file__ to find the sibling
    vendored module. Search order:

      1. Normal `import vendored_sx126x` (works when the dir is on sys.path,
         e.g. during development when rnsd is launched from the project dir).
      2. The standard Reticulum custom-interfaces directory at
         ~/.reticulum/interfaces/vendored_sx126x.py.
      3. Any path listed in the RETICULUM_HAT_MOD_DIR environment variable.
      4. The current working directory.
    """
    # 1. Normal import (works if dir is on sys.path)
    try:
        import vendored_sx126x  # noqa: F401
        return vendored_sx126x
    except ImportError:
        pass

    candidates = []

    # 2. Standard Reticulum interfaces directory
    candidates.append(os.path.expanduser("~/.reticulum/interfaces/vendored_sx126x.py"))

    # 3. Env-var override
    env_dir = os.environ.get("RETICULUM_HAT_MOD_DIR")
    if env_dir:
        candidates.append(os.path.join(env_dir, "vendored_sx126x.py"))

    # 4. Current working directory
    candidates.append(os.path.join(os.getcwd(), "vendored_sx126x.py"))

    for path in candidates:
        if path and os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("vendored_sx126x", path)
            if spec is None or spec.loader is None:
                continue
            mod = importlib.util.module_from_spec(spec)
            try:
                spec.loader.exec_module(mod)
            except Exception as e:
                raise ImportError(f"Failed to exec vendored_sx126x at {path}: {e}")
            sys.modules["vendored_sx126x"] = mod
            return mod

    raise ImportError(
        "Could not locate vendored_sx126x.py. Tried: " +
        ", ".join(p for p in candidates if p) +
        ". Place it next to SX126xInterface.py in ~/.reticulum/interfaces/ " +
        "(or set RETICULUM_HAT_MOD_DIR to the directory containing both files)."
    )


# ---------------------------------------------------------------------------
# Bundled platform profiles (SBC-level pin/GPIO/SPIscheme description).
#
# A platform profile describes the SBC's GPIO/SPI addressing scheme, NOT any
# specific radio wiring. The `header_pin_to_line` table maps physical 40-pin
# header pin numbers to the (gpiochip, line_offset) for THIS specific SBC.
# This is the bridge that lets a HAT profile be defined purely in physical
# pin numbers (portable across SBCs with the same header layout) while
# remaining runnable on each individual platform via this translation table.
#
# Populate the table on-demand as new HATs are added — only the physical
# pins actually used by known HATs need entries. Don't try to be exhaustive
# up front.
# ---------------------------------------------------------------------------

PLATFORM_PROFILES = {
    "raspberry-pi": {
        "platform_name":    "raspberry-pi",
        "platform_version": 1,
        "gpiochip":         "gpiochip0",
        "spi_bus_default":  0,
        # Physical 40-pin header pin -> (gpiochip, line_offset). BCM GPIO
        # numbers ARE the libgpiod line offsets on gpiochip0, so the value
        # column here is just the BCM GPIO number for that physical pin.
        # Only pins used by known HATs are listed.
        "header_pin_to_line": {
            11: ("gpiochip0", 17),   # physical 11 = BCM17
            12: ("gpiochip0", 18),   # physical 12 = BCM18  (MeshAdv RESET)
            13: ("gpiochip0", 27),   # physical 13 = BCM27
            15: ("gpiochip0", 22),   # physical 15 = BCM22
            16: ("gpiochip0", 23),   # physical 16 = BCM23
            18: ("gpiochip0", 24),   # physical 18 = BCM24
            22: ("gpiochip0", 25),   # physical 22 = BCM25
            29: ("gpiochip0",  5),   # physical 29 = BCM5
            31: ("gpiochip0",  6),   # physical 31 = BCM6
            32: ("gpiochip0", 12),   # physical 32 = BCM12  (MeshAdv RXEN)
            33: ("gpiochip0", 13),   # physical 33 = BCM13  (MeshAdv TXEN)
            35: ("gpiochip0", 19),   # physical 35 = BCM19
            36: ("gpiochip0", 16),   # physical 36 = BCM16  (MeshAdv IRQ)
            37: ("gpiochip0", 26),   # physical 37 = BCM26
            38: ("gpiochip0", 20),   # physical 38 = BCM20  (MeshAdv BUSY)
            40: ("gpiochip0", 21),   # physical 40 = BCM21  (MeshAdv CS — but
                                      # CS is driven by SPI CE, not GPIO)
        },
        "notes": (
            "Standard Raspberry Pi 3/4/5 40-pin header. BCM GPIO numbers "
            "are the libgpiod line offsets on gpiochip0, so the table just "
            "uses the BCM number per physical pin. "
            "Re: chip-select on the MeshAdv Pi HAT (CS=BCM21 / physical 40): "
            "this is NOT a native SPI0 CE line. The Pi's typical SPI overlay "
            "is `dtoverlay=spi0-0cs` (zero hardware CS lines) — verified on "
            "Pi OS Bookworm on a real Pi Zero 2W. In that configuration "
            "spidev's hardware-CE toggling has no physical pin to drive and "
            "the SX126x chip never sees CS, so the driver bit-bangs CS via "
            "libgpiod on BCM21 instead. If a user prefers to use the "
            "controller's hardware-CE for CS, they can switch the overlay "
            "to `dtoverlay=spi0-1cs,cs0_pin=21` (routes CE0 onto GPIO21) "
            "and set `header_pin_cs = -1` in the board profile — but the "
            "bit-bang path works without any /boot/firmware/config.txt edit "
            "or reboot, so it is the default."
        ),
    },

    "luckfox-pico": {
        "platform_name":    "luckfox-pico",
        "platform_version": 1,
        "gpiochip":         "gpiochip1",
        "spi_bus_default":  0,
        # Femtofox's real, hardware-confirmed pins. The line offsets on
        # gpiochip1 are ABSOLUTE Rockchip GPIO numbers (not BCM). Values
        # confirmed against the actual femtofox SX1262 hat wiring.
        "header_pin_to_line": {
             6: ("gpiochip1", 16),   # physical 6   = GPIO48 (line 16)   CS/NSS
            12: ("gpiochip1", 24),   # physical 12  = GPIO56 (line 24)   RXEN
            13: ("gpiochip1", 25),   # physical 13  = GPIO57 (line 25)   RESET
            16: ("gpiochip1", 22),   # physical 16  = GPIO54 (line 22)   BUSY
            17: ("gpiochip1", 23),   # physical 17  = GPIO55 (line 23)   IRQ
        },
        "notes": (
            "Femtofox / Luckfox Pico Pro. The line offsets on gpiochip1 "
            "are absolute Rockchip GPIO numbers, not BCM. Only the pins "
            "actually wired on Femtofox's integrated SX1262 hat are listed; "
            "add entries here when new HATs require other pins."
        ),
    },
}


# ---------------------------------------------------------------------------
# Bundled radio board / HAT profiles (electronics-only description of the
# radio board's wiring, in terms of physical 40-pin-header pin numbers).
#
# These deliberately contain NO frequency / power / regional settings — those
# are user-facing policy and live in the interface config block.
#
# -1 for any header_pin_* means "not wired / not needed" (e.g. an E22 module
# drives its RF switch via DIO2 internally, so TXEN/RXEN as external GPIOs
# may be left unwired).
#
# `spi_bus` and `spi_cs` are still needed alongside the header-pin GPIO map
# because the SPI MOSI/MISO/CLK/CS routing through spidev is a separate
# concern from the chip's control GPIOs.
# ---------------------------------------------------------------------------

BOARD_PROFILES = {
    "meshadv-pi-hat-v1.1": {
        "board_name":        "meshadv-pi-hat-v1.1",
        "profile_version":   2,
        # CS / NSS: physical pin 40 → BCM21. The MeshAdv HAT does NOT have
        # its CS pin wired to the Pi's native SPI0 CE0 (GPIO8). The Pi's
        # device tree overlay is typically `dtoverlay=spi0-0cs` (zero
        # hardware chip-select lines), in which case spidev's hardware-CE
        # toggling has no physical pin to drive and the chip never sees
        # CS. So the driver MUST bit-bang CS via libgpiod on this GPIO.
        "header_pin_cs":     40,
        "header_pin_irq":    36,    # physical pin 36 → BCM16 (gpiochip0 line 16)
        "header_pin_busy":   38,    # physical pin 38 → BCM20 (line 20)
        "header_pin_reset":  12,    # physical pin 12 → BCM18 (line 18)
        "header_pin_txen":   33,    # physical pin 33 → BCM13 (line 13)
        "header_pin_rxen":   32,    # physical pin 32 → BCM12 (line 12)
        "spi_bus":           0,
        "spi_cs":            0,
        "dio2_rf_switch":    True,
        "dio3_tcxo_voltage": 1.8,
        "tcxo_delay_ms":     5.0,
        "txpower_max":       22,
        "rx_boosted_gain":   True,
        "profile_notes": (
            "MeshAdv Pi HAT v1.1 with Ebyte E22-900M30S SX1262 module. "
            "E22 module drives its RF switch via DIO2, so set_dio2_as_rf_"
            "switch_ctrl is enabled and the external TXEN/RXEN GPIOs are "
            "wired but optional. Module uses a TCXO on DIO3 at 1.8V. "
            "CS/NSS is wired to BCM21 (physical pin 40), which is NOT a "
            "native SPI0 CE line — the Pi's dtoverlay is typically "
            "`spi0-0cs` (zero hardware CS lines), so the driver bit-bangs "
            "CS via libgpiod on this GPIO instead of relying on spidev's "
            "hardware-CE (which would toggle a pin that goes nowhere). "
            "This was verified on a real Raspberry Pi Zero 2W running "
            "Raspberry Pi OS Bookworm with `dtoverlay=spi0-0cs`."
        ),
    },

    "femtofox-integrated-v1": {
        "board_name":        "femtofox-integrated-v1",
        "profile_version":   1,
        "header_pin_cs":     -1,    # CS handled by spidev via SPI CE
        "header_pin_irq":    17,    # physical pin 17 → GPIO55 (gpiochip1 line 23)
        "header_pin_busy":   16,    # physical pin 16 → GPIO54 (line 22)
        "header_pin_reset":  13,    # physical pin 13 → GPIO57 (line 25)
        "header_pin_txen":   -1,    # TXEN bridged to DIO2 on the module, not a GPIO
        "header_pin_rxen":   12,    # physical pin 12 → GPIO56 (line 24)
        "spi_bus":           0,
        "spi_cs":            0,
        "dio2_rf_switch":    True,
        "dio3_tcxo_voltage": 1.8,
        "tcxo_delay_ms":     5.0,
        "txpower_max":       22,
        "rx_boosted_gain":   True,
        "profile_notes": (
            "Femtofox's integrated SX1262 hat. Hardware-verified. Module "
            "has no separate TXEN GPIO (TXEN is bridged to DIO2 internally), "
            "so header_pin_txen is -1 and the driver relies on DIO2-as-RF-"
            "switch control. Module uses a TCXO on DIO3 at 1.8V."
        ),
    },

    "station-g3": {
        "board_name":        "station-g3",
        "profile_version":   1,
        # BQ/Uniteng Station G3 devkit, Raspberry Pi Zero 2W MCU
        # daughterboard path, Primary RF Slot (populated by default with
        # the BQ35LORA900V1M SX1262 module). NOT YET hardware-verified -
        # derived from BQ's own published meshtasticd YAML example config,
        # a live `pinctrl get 7-25` dump from BQ's own test rig, and
        # rep-provided lna_control.sh/pa_control.sh scripts. See
        # Reticulum-StationG3/HARDWARE-RECON.md for the full derivation.
        "header_pin_cs":     -1,   # SPI0 CE0 (BCM8/physical 24) - native
                                   # spidev hardware CS, not bit-banged.
        "header_pin_irq":    15,  # physical pin 15 -> BCM22 (DIO1/IRQ,
                                   # per BQ's meshtasticd config: IRQ: 22)
        "header_pin_busy":   18,  # physical pin 18 -> BCM24 (BUSY,
                                   # per BQ's meshtasticd config: Busy: 24)
        "header_pin_reset":  36,  # physical pin 36 -> BCM16 (RESET,
                                   # per BQ's meshtasticd config: Reset: 16)
        "header_pin_txen":   11,  # physical pin 11 -> BCM17 = PA enable.
                                   # Rep-confirmed "pin 11 PA Mode"; matches
                                   # pa_control.sh's GPIO_PIN=17 exactly.
                                   # Active-HIGH (PAHIGH -> gpioset ...=1),
                                   # matching the driver's existing txen
                                   # assumption - no polarity flag needed.
        "header_pin_rxen":   16,  # physical pin 16 -> BCM23 = LNA enable.
                                   # Rep-confirmed "pin 16 Primary Slot LNA
                                   # Mode"; matches lna_control.sh's
                                   # GPIO_PIN=23 exactly. Active-LOW
                                   # (LNAON -> gpioset ...=0) - OPPOSITE of
                                   # the driver's historical rxen
                                   # assumption, hence rxen_active_low.
        "txen_active_low":   False,
        "rxen_active_low":   True,
        "spi_bus":           0,
        "spi_cs":            0,
        "dio2_rf_switch":    True,
        "dio3_tcxo_voltage": 1.8,   # ASSUMED by analogy with the ESP32-S3
                                    # daughterboard path's confirmed spec
                                    # (same BQ35LORA900V1M RF module either
                                    # way) - not independently confirmed
                                    # for the RPi path's exact YAML value.
        "tcxo_delay_ms":     5.0,
        # SAFETY CAP, not yet a calibrated limit: capped low (chip-level 7dBm,
        # not the chip's real max of 22dBm) to keep actual RF output (after
        # the external PA) comfortably under Station G3's documented ~2W
        # false-OVP-trip threshold (see Reticulum-StationG3/HARDWARE-RECON.md)
        # until real hardware arrives and the PA gain curve for THIS specific
        # RPi-Zero-2W daughterboard path can be measured. Basis: the ESP32-S3
        # daughterboard path's own reference config uses chip-level 7dBm to
        # reach ~27dBm/0.5W actual (implying ~20dB of PA gain on this same
        # BQ35LORA900V1M module) - reusing that same chip-level value here
        # gives an actual-output estimate safely under the 1W/30dBm target
        # even if this path's PA gain turns out somewhat higher than assumed.
        # This value is NOT currently enforced automatically elsewhere in this
        # file for other profiles (txpower_max was previously informational
        # only) - see the new hard-cap check added in __init__ below, which
        # now clamps user-configured `txpower` to this profile's txpower_max
        # for every board, not just this one.
        "txpower_max":       7,
        "rx_boosted_gain":   True,
        "profile_notes": (
            "BQ/Uniteng Station G3, Raspberry Pi Zero 2W daughterboard, "
            "Primary RF Slot (BQ35LORA900V1M / SX1262). NOT YET hardware-"
            "verified - pin mapping derived from BQ's own published docs "
            "and scripts, not confirmed on a physical board. Requires "
            "`dtparam=i2c_arm=on`, `dtoverlay=spi0-1cs`, "
            "`dtoverlay=spi1-1cs` in /boot/firmware/config.txt per BQ's "
            "own troubleshooting notes. Station G3 adds software-"
            "controllable PA/LNA enable GPIOs over Station G2, which only "
            "has physical jumpers for this - the underlying LoRa/SPI "
            "pinout is otherwise identical between G2 and G3 (rep-"
            "confirmed firmware compatibility). See "
            "Reticulum-StationG3/HARDWARE-RECON.md for the full recon."
        ),
    },

    "generic-sx1262-manual": {
        "board_name":        "generic-sx1262-manual",
        "profile_version":   1,
        # These placeholder physical pins are intentionally chosen to be
        # plausible but NOT necessarily correct for any real board. Copy
        # this profile and edit the values for your actual wiring.
        "header_pin_cs":     -1,
        "header_pin_irq":    11,    # PLACEHOLDER
        "header_pin_busy":   13,    # PLACEHOLDER
        "header_pin_reset":   7,    # PLACEHOLDER (Pi physical 7 = BCM4)
        "header_pin_txen":   15,    # PLACEHOLDER
        "header_pin_rxen":   16,    # PLACEHOLDER
        "spi_bus":           0,
        "spi_cs":            0,
        "dio2_rf_switch":    True,
        "dio3_tcxo_voltage": 1.8,
        "tcxo_delay_ms":     5.0,
        "txpower_max":       22,
        "rx_boosted_gain":   True,
        "profile_notes": (
            "TEMPLATE PROFILE. The header_pin_* values are PLACEHOLDERS and "
            "do not correspond to any real board. Copy this profile into a "
            "user overlay (sx126x_boards) or adapt for your hand-wired "
            "SX1262 module. Use physical 40-pin-header pin numbers (1..40)."
        ),
    },
}


# Pin field names that participate in header-pin resolution.
_PIN_FIELDS = ("cs", "irq", "busy", "reset", "txen", "rxen")

# Special board name that triggers the escape hatch.
_CUSTOM_BOARD_NAME = "custom"


class _ProfileResolutionError(ValueError):
    """Raised for any error in platform/board profile resolution."""


class _ProfileResolver:
    """Loads bundled + overlay profiles, resolves a (platform, board) pair
    (plus any config-level overrides) into a flat dict of concrete values
    the rest of the interface can consume.

    Resolution result fields:
        platform_name, platform_version, platform_notes
        board_name, board_version, board_notes   ("custom" if escape hatch)
        gpiochip, spi_bus, spi_cs
        pin_lines: {pin_field_name: (gpiochip, line_offset) | None}
        dio2_rf_switch, dio3_tcxo_voltage, tcxo_delay_ms
        rx_boosted_gain, txpower_max
        overrides_used: list of (key, profile_value, override_value)
        used_profile_mode: True if the profile system was used, False if
                          the legacy direct-gpiochip-line mode was used
                          (no platform/radio_board keys in config).
    """

    def __init__(self):
        # Deep-copy the bundled dicts so overlay merges don't mutate them.
        self.platforms = copy.deepcopy(PLATFORM_PROFILES)
        self.boards    = copy.deepcopy(BOARD_PROFILES)
        self._load_overlays()

    # -----------------------------------------------------------------
    # Overlay file loading (sx126x_platforms / sx126x_boards)
    # -----------------------------------------------------------------

    def _overlay_search_paths(self, filename):
        candidates = []
        # Standard Reticulum custom-interfaces directory
        candidates.append(os.path.expanduser(f"~/.reticulum/interfaces/{filename}"))
        # Env-var override
        env_dir = os.environ.get("RETICULUM_HAT_MOD_DIR")
        if env_dir:
            candidates.append(os.path.join(env_dir, filename))
        # Current working directory
        candidates.append(os.path.join(os.getcwd(), filename))
        return candidates

    def _load_overlays(self):
        """Look for sx126x_platforms and sx126x_boards files and merge
        any user-defined profiles into the bundled dicts. Failures here
        are logged as WARNINGs but do not prevent the interface from
        using just the bundled profiles."""
        try:
            from RNS.vendor.configobj import ConfigObj
        except Exception:
            RNS.log(
                "Could not import RNS.vendor.configobj; skipping user "
                "sx126x_platforms / sx126x_boards overlays.",
                RNS.LOG_WARNING,
            )
            return

        for filename, target_name in (
            ("sx126x_platforms", "platforms"),
            ("sx126x_boards",    "boards"),
        ):
            path = None
            for candidate in self._overlay_search_paths(filename):
                if os.path.isfile(candidate):
                    path = candidate
                    break
            if path is None:
                continue
            try:
                self._merge_overlay_file(path, target_name)
                RNS.log(
                    f"Loaded SX126x overlay from {path}", RNS.LOG_VERBOSE,
                )
            except Exception as e:
                RNS.log(
                    f"Failed to apply SX126x overlay {path}: {e}",
                    RNS.LOG_WARNING,
                )

    def _merge_overlay_file(self, path, target_name):
        from RNS.vendor.configobj import ConfigObj

        cfg = ConfigObj(path)
        section = cfg.get(target_name)
        if not section:
            return

        target = self.platforms if target_name == "platforms" else self.boards

        for profile_name, sub in section.items():
            if not isinstance(sub, dict):
                continue
            overlay = {}
            if "based_on" in sub:
                overlay["based_on"] = str(sub["based_on"])
            for k, v in sub.items():
                if k == "based_on":
                    continue
                if k == "header_pin_to_line":
                    overlay[k] = self._parse_pin_map(v)
                else:
                    overlay[k] = self._parse_scalar(v)
            self._merge_profile(target, profile_name, overlay)

    @staticmethod
    def _parse_pin_map(value):
        """Parse a header_pin_to_line value from an overlay file.

        Accepts either:
          * a dict (already-parsed ConfigObj sub-section)
          * a JSON object string: '{"11": ["gpiochip0", 17], ...}'
        Returns a {int: (str, int)} dict.
        """
        if isinstance(value, dict):
            raw = value
        else:
            try:
                raw = json.loads(str(value))
            except Exception as e:
                raise _ProfileResolutionError(
                    f"header_pin_to_line must be a JSON object string, "
                    f"got {value!r}: {e}"
                )
        result = {}
        for k, v in raw.items():
            try:
                pin = int(k)
                chip = str(v[0])
                line = int(v[1])
            except Exception as e:
                raise _ProfileResolutionError(
                    f"Bad entry in header_pin_to_line: {k} -> {v!r}: {e}"
                )
            result[pin] = (chip, line)
        return result

    @staticmethod
    def _parse_scalar(value):
        """Coerce a scalar ConfigObj value to int/float/bool/str sensibly."""
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value
        s = str(value).strip()
        if s.lower() in ("true", "yes", "on"):
            return True
        if s.lower() in ("false", "no", "off"):
            return False
        try:
            if "." in s:
                return float(s)
            return int(s)
        except ValueError:
            pass
        return s

    def _merge_profile(self, target, profile_name, overlay):
        based_on = overlay.pop("based_on", None)
        if based_on is not None:
            if based_on not in target:
                raise _ProfileResolutionError(
                    f"Overlay profile {profile_name!r} references unknown "
                    f"based_on={based_on!r}. Known: {sorted(target.keys())}"
                )
            new = copy.deepcopy(target[based_on])
        else:
            new = {}
        for k, v in overlay.items():
            new[k] = v
        target[profile_name] = new

    # -----------------------------------------------------------------
    # Resolution
    # -----------------------------------------------------------------

    def resolve(self, config, log_fn=None):
        """Resolve the final concrete config from the interface config block.

        `config` is the ConfigObj for this interface block (dict-like).
        `log_fn` is an optional callable(msg, level) used for the WARNING
        when a per-key override is detected. If None, defaults to RNS.log.
        """
        if log_fn is None:
            def log_fn(msg, level):
                RNS.log(msg, level)

        config_keys = set(config.keys())
        platform_name = str(config["platform"]) if "platform" in config else None
        board_name    = str(config["radio_board"]) if "radio_board" in config else None

        profile_mode = ("platform" in config) or ("radio_board" in config)

        # ------------------------------------------------------------------
        # Legacy direct-gpiochip-line mode (backward compatibility for old
        # configs that don't set platform / radio_board).
        # ------------------------------------------------------------------
        if not profile_mode:
            return self._resolve_legacy(config, log_fn)

        # ------------------------------------------------------------------
        # Profile-based mode.
        # ------------------------------------------------------------------
        if platform_name is None:
            raise _ProfileResolutionError(
                "radio_board was set but platform was not. Either set both, "
                "or remove both to use the legacy direct-gpiochip-line mode."
            )
        if board_name is None:
            raise _ProfileResolutionError(
                "platform was set but radio_board was not. Either set both, "
                "or remove both to use the legacy direct-gpiochip-line mode."
            )

        if platform_name not in self.platforms:
            raise _ProfileResolutionError(
                f"Unknown platform {platform_name!r}. Known platforms: "
                f"{sorted(self.platforms.keys())}"
            )

        # Custom escape hatch: the user provides gpiochip + line offsets
        # directly. Skip board-profile resolution entirely.
        if board_name == _CUSTOM_BOARD_NAME:
            return self._resolve_custom(platform_name, config, log_fn)

        if board_name not in self.boards:
            raise _ProfileResolutionError(
                f"Unknown radio_board {board_name!r}. Known boards: "
                f"{sorted(self.boards.keys())}"
            )

        return self._resolve_profile(platform_name, board_name, config, log_fn)

    def _resolve_profile(self, platform_name, board_name, config, log_fn):
        platform = self.platforms[platform_name]
        board    = self.boards[board_name]

        overrides_used = []

        # Resolve each header_pin_* to (gpiochip, line_offset), applying
        # per-key overrides from config (interpreted as physical pin numbers
        # that get re-resolved via the platform's pin map).
        pin_lines = {}
        for pin_field in _PIN_FIELDS:
            profile_key  = "header_pin_" + pin_field
            override_key = "pin_"          + pin_field

            header_pin = board[profile_key]
            if override_key in config:
                override_val = int(config[override_key])
                if override_val != header_pin:
                    overrides_used.append((override_key, header_pin, override_val))
                header_pin = override_val

            if header_pin < 0:
                pin_lines[pin_field] = None
                continue

            pin_map = platform["header_pin_to_line"]
            if header_pin not in pin_map:
                supported = sorted(pin_map.keys())
                raise _ProfileResolutionError(
                    f"Board {board_name!r} requires physical pin {header_pin} "
                    f"for {profile_key}, but platform {platform_name!r} has no "
                    f"header_pin_to_line entry for that pin. "
                    f"Supported physical pins on {platform_name!r}: {supported}. "
                    f"Either add the pin to the platform profile "
                    f"(via a user overlay), use radio_board=custom with "
                    f"explicit {override_key} as a gpiochip line offset, or "
                    f"choose a different board / platform combination."
                )
            pin_lines[pin_field] = pin_map[header_pin]

        # spi_bus / spi_cs
        spi_bus = int(board["spi_bus"])
        spi_cs  = int(board["spi_cs"])
        if "spi_bus" in config and int(config["spi_bus"]) != spi_bus:
            overrides_used.append(("spi_bus", spi_bus, int(config["spi_bus"])))
            spi_bus = int(config["spi_bus"])
        if "spi_cs" in config and int(config["spi_cs"]) != spi_cs:
            overrides_used.append(("spi_cs", spi_cs, int(config["spi_cs"])))
            spi_cs = int(config["spi_cs"])

        # gpiochip — defaults from platform, override allowed
        gpiochip = str(platform["gpiochip"])
        if "gpiochip" in config and str(config["gpiochip"]) != gpiochip:
            overrides_used.append(("gpiochip", gpiochip, str(config["gpiochip"])))
            gpiochip = str(config["gpiochip"])

        # Radio-electronics fields — defaults from board, override allowed
        dio2_rf_switch    = self._bool_or(board.get("dio2_rf_switch", True), True)
        dio3_tcxo_voltage = self._float_or(board.get("dio3_tcxo_voltage", 1.8), 1.8)
        tcxo_delay_ms     = self._float_or(board.get("tcxo_delay_ms", 5.0), 5.0)
        txpower_max       = self._int_or(board.get("txpower_max", 22), 22)
        rx_boosted_gain   = self._bool_or(board.get("rx_boosted_gain", True), True)
        # Polarity of the external TXEN/RXEN "enabled" state. Almost every
        # board so far is active-HIGH for both (the historical assumption
        # baked into the driver), but the Station G3's LNA-enable pin is
        # active-LOW (LNA ON = logic 0, per BQ's own lna_control.sh) - so
        # this can no longer be assumed true for every board.
        txen_active_low   = self._bool_or(board.get("txen_active_low", False), False)
        rxen_active_low   = self._bool_or(board.get("rxen_active_low", False), False)

        for key, parser, current in (
            ("dio2_rf_switch",    self._bool_or, dio2_rf_switch),
            ("dio3_tcxo_voltage", self._float_or, dio3_tcxo_voltage),
            ("tcxo_delay_ms",     self._float_or, tcxo_delay_ms),
            ("txpower_max",       self._int_or, txpower_max),
            ("rx_boosted_gain",   self._bool_or, rx_boosted_gain),
            ("txen_active_low",   self._bool_or, txen_active_low),
            ("rxen_active_low",   self._bool_or, rxen_active_low),
        ):
            if key in config:
                raw = config[key]
                new_val = parser(raw, current)
                if new_val != current:
                    overrides_used.append((key, current, new_val))
                    if   parser is self._bool_or:  dio2_rf_switch = new_val if key == "dio2_rf_switch" else dio2_rf_switch
                    if   parser is self._bool_or:  rx_boosted_gain   = new_val if key == "rx_boosted_gain" else rx_boosted_gain
                    if   parser is self._bool_or:  txen_active_low   = new_val if key == "txen_active_low" else txen_active_low
                    if   parser is self._bool_or:  rxen_active_low   = new_val if key == "rxen_active_low" else rxen_active_low
                    if   parser is self._float_or: dio3_tcxo_voltage = new_val if key == "dio3_tcxo_voltage" else dio3_tcxo_voltage
                    if   parser is self._float_or: tcxo_delay_ms     = new_val if key == "tcxo_delay_ms" else tcxo_delay_ms
                    if   parser is self._int_or:   txpower_max       = new_val if key == "txpower_max" else txpower_max

        return {
            "used_profile_mode": True,
            "platform_name":     platform_name,
            "platform_version":  platform["platform_version"],
            "platform_notes":    platform["notes"],
            "board_name":        board_name,
            "board_version":     board["profile_version"],
            "board_notes":       board["profile_notes"],
            "gpiochip":          gpiochip,
            "spi_bus":           spi_bus,
            "spi_cs":            spi_cs,
            "pin_lines":         pin_lines,
            "dio2_rf_switch":    dio2_rf_switch,
            "dio3_tcxo_voltage": dio3_tcxo_voltage,
            "tcxo_delay_ms":     tcxo_delay_ms,
            "txpower_max":       txpower_max,
            "rx_boosted_gain":   rx_boosted_gain,
            "txen_active_low":   txen_active_low,
            "rxen_active_low":   rxen_active_low,
            "overrides_used":    overrides_used,
        }

    def _resolve_custom(self, platform_name, config, log_fn):
        """Escape hatch: user provides gpiochip + line offsets directly."""
        if platform_name not in self.platforms:
            raise _ProfileResolutionError(
                f"Unknown platform {platform_name!r} (in custom mode). "
                f"Known platforms: {sorted(self.platforms.keys())}"
            )

        platform = self.platforms[platform_name]
        gpiochip = str(config["gpiochip"]) if "gpiochip" in config else str(platform["gpiochip"])
        spi_bus  = int(config["spi_bus"])   if "spi_bus"  in config else int(platform["spi_bus_default"])
        spi_cs   = int(config["spi_cs"])    if "spi_cs"   in config else 0

        pin_lines = {}
        for pin_field in _PIN_FIELDS:
            key = "pin_" + pin_field
            if key not in config:
                pin_lines[pin_field] = None
                continue
            line = int(config[key])
            if line < 0:
                pin_lines[pin_field] = None
            else:
                pin_lines[pin_field] = (gpiochip, line)

        # Defaults for radio-electronics fields, override-able per-key
        dio2_rf_switch    = self._bool_or(config.get("dio2_rf_switch", True), True)
        dio3_tcxo_voltage = self._float_or(config.get("dio3_tcxo_voltage", 1.8), 1.8)
        tcxo_delay_ms     = self._float_or(config.get("tcxo_delay_ms", 5.0), 5.0)
        rx_boosted_gain   = self._bool_or(config.get("rx_boosted_gain", True), True)
        txpower_max       = self._int_or(config.get("txpower_max", 22), 22)

        return {
            "used_profile_mode": True,
            "platform_name":     platform_name,
            "platform_version":  platform["platform_version"],
            "platform_notes":    platform["notes"],
            "board_name":        _CUSTOM_BOARD_NAME,
            "board_version":     0,
            "board_notes":       "Escape-hatch mode: pin_* values are direct gpiochip line offsets.",
            "gpiochip":          gpiochip,
            "spi_bus":           spi_bus,
            "spi_cs":            spi_cs,
            "pin_lines":         pin_lines,
            "dio2_rf_switch":    dio2_rf_switch,
            "dio3_tcxo_voltage": dio3_tcxo_voltage,
            "tcxo_delay_ms":     tcxo_delay_ms,
            "txpower_max":       txpower_max,
            "rx_boosted_gain":   rx_boosted_gain,
            "overrides_used":    [],   # custom mode has no profile to override
        }

    def _resolve_legacy(self, config, log_fn):
        """Legacy mode: pin_* are direct gpiochip line offsets (no profiles).

        Default platform is raspberry-pi just to source a default gpiochip
        and spi_bus. The pin values themselves are taken verbatim from
        config (or the historical hardcoded MeshAdv Pi HAT defaults)."""
        platform_name = "raspberry-pi"   # assumed for the default gpiochip
        platform = self.platforms[platform_name]

        gpiochip = str(config["gpiochip"]) if "gpiochip" in config else str(platform["gpiochip"])
        spi_bus  = int(config["spi_bus"])   if "spi_bus"  in config else int(platform["spi_bus_default"])
        spi_cs   = int(config["spi_cs"])    if "spi_cs"   in config else 0

        # The hardcoded historical defaults for a Pi + MeshAdv HAT v1.1
        legacy_defaults = {"irq": 16, "busy": 20, "reset": 18, "txen": 13, "rxen": 12}
        pin_lines = {}
        for pin_field in _PIN_FIELDS:
            key = "pin_" + pin_field
            if key in config:
                line = int(config[key])
            elif pin_field in legacy_defaults:
                line = legacy_defaults[pin_field]
            else:
                line = -1
            pin_lines[pin_field] = None if line < 0 else (gpiochip, line)

        dio2_rf_switch    = self._bool_or(config.get("dio2_rf_switch", True), True)
        dio3_tcxo_voltage = self._float_or(config.get("dio3_tcxo_voltage", 1.8), 1.8)
        tcxo_delay_ms     = self._float_or(config.get("tcxo_delay_ms", 5.0), 5.0)
        rx_boosted_gain   = self._bool_or(config.get("rx_boosted_gain", True), True)
        txpower_max       = self._int_or(config.get("txpower_max", 22), 22)

        log_fn(
            "SX126xInterface legacy mode: no platform/radio_board set in "
            "config; pin_irq/pin_busy/pin_reset/pin_txen/pin_rxen are being "
            "interpreted as direct gpiochip line offsets (BCM numbers on a "
            "Pi). Migrate by adding `platform = raspberry-pi` and "
            "`radio_board = meshadv-pi-hat-v1.1` to your interface config.",
            RNS.LOG_WARNING,
        )

        return {
            "used_profile_mode": False,
            "platform_name":     platform_name,
            "platform_version":  platform["platform_version"],
            "platform_notes":    "(legacy mode — no profile selected)",
            "board_name":        "(legacy)",
            "board_version":     0,
            "board_notes":       "(legacy mode — no profile selected)",
            "gpiochip":          gpiochip,
            "spi_bus":           spi_bus,
            "spi_cs":            spi_cs,
            "pin_lines":         pin_lines,
            "dio2_rf_switch":    dio2_rf_switch,
            "dio3_tcxo_voltage": dio3_tcxo_voltage,
            "tcxo_delay_ms":     tcxo_delay_ms,
            "txpower_max":       txpower_max,
            "rx_boosted_gain":   rx_boosted_gain,
            "overrides_used":    [],
        }

    # -----------------------------------------------------------------
    # Tiny value-parsing helpers
    # -----------------------------------------------------------------

    @staticmethod
    def _bool_or(value, default):
        if isinstance(value, bool):
            return value
        s = str(value).strip().lower()
        if s in ("true", "yes", "on", "1"):
            return True
        if s in ("false", "no", "off", "0"):
            return False
        return default

    @staticmethod
    def _int_or(value, default):
        try:
            return int(str(value).strip())
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _float_or(value, default):
        try:
            return float(str(value).strip())
        except (TypeError, ValueError):
            return default


class SX126xInterface(Interface):
    # Defaults matching the Reticulum custom-interface contract
    DEFAULT_IFAC_SIZE = 8
    HW_MTU            = 508

    FREQ_MIN = 137000000
    FREQ_MAX = 1020000000

    # SX1262 maximum single LoRa frame payload
    LORA_MAX_PAYLOAD  = 255
    # We prepend a 1-byte split-frame header, so usable per frame
    FRAME_PAYLOAD_MAX = 254

    # Split-packet framing flags (RNode-compatible)
    FLAG_SPLIT = 0x01

    # Sliding-window airtime accounting
    AIRTIME_SHORT_WINDOW = 15.0           # seconds
    AIRTIME_LONG_WINDOW  = 60.0 * 60.0    # 1 hour

    # TX queue bound — we drop packets if the queue is full rather than
    # block the Reticulum Transport thread.
    TX_QUEUE_MAX = 64

    # Radio lockup recovery thresholds
    RADIO_REINIT_THRESHOLD = 5   # consecutive SPI failures -> attempt reinit
    RADIO_GIVEUP_THRESHOLD = 3   # consecutive failed reinits -> mark offline

    def __init__(self, owner, configuration):
        # Vendored driver load first — fail early with a clear message if
        # the driver file is missing or malformed.
        try:
            self.vd = _load_vendored_driver()
        except ImportError as e:
            RNS.log("SX126xInterface could not load vendored_sx126x driver: " + str(e), RNS.LOG_CRITICAL)
            RNS.panic()
            raise

        super().__init__()

        # Parse configuration through the standard helper
        c = Interface.get_config_obj(configuration)
        name = c["name"]

        # Set self.name early so __str__ works for the resolver's log output
        self.name = name

        # ------------------------------------------------------------------
        # Resolve platform/board → concrete pin/GPIO/SPI values
        # ------------------------------------------------------------------
        try:
            resolver = _ProfileResolver()
            resolution = resolver.resolve(c)
        except _ProfileResolutionError as e:
            RNS.log(str(self) + " profile resolution failed: " + str(e), RNS.LOG_ERROR)
            raise

        # Unpack the resolved values
        self.resolution        = resolution
        self.platform_name     = resolution["platform_name"]
        self.platform_version  = resolution["platform_version"]
        self.board_name        = resolution["board_name"]
        self.board_version     = resolution["board_version"]
        self.gpiochip          = resolution["gpiochip"]
        self.spi_bus           = resolution["spi_bus"]
        self.spi_cs            = resolution["spi_cs"]
        self.dio2_rf_switch    = resolution["dio2_rf_switch"]
        self.dio3_tcxo_voltage = resolution["dio3_tcxo_voltage"]
        self.tcxo_delay_ms     = resolution["tcxo_delay_ms"]
        self.txpower_max       = resolution["txpower_max"]
        self.rx_boosted_gain   = resolution["rx_boosted_gain"]
        self.txen_active_low   = resolution.get("txen_active_low", False)
        self.rxen_active_low   = resolution.get("rxen_active_low", False)
        # pin_lines is {pin_field_name: (gpiochip, line) | None}
        self.pin_lines = resolution["pin_lines"]
        # Convenience accessor for the legacy attribute name (used in
        # _init_radio etc.) — stores the line-offset integer or -1 if absent.
        def _line_of(field):
            v = self.pin_lines.get(field)
            return -1 if v is None else v[1]
        self.pin_cs    = _line_of("cs")
        self.pin_irq   = _line_of("irq")
        self.pin_busy  = _line_of("busy")
        self.pin_reset = _line_of("reset")
        self.pin_txen  = _line_of("txen")
        self.pin_rxen  = _line_of("rxen")
        # Per-pin gpiochip path, since some boards (e.g. the Luckfox Lyra
        # Zero W) wire different control pins to DIFFERENT gpiochips - the
        # single shared `self.gpiochip` default is only correct for
        # single-chip boards (Raspberry Pi, femtofox). Missing/unwired
        # pins fall back to the platform default so single-chip boards are
        # unaffected. Discovered live: passing only the single default
        # gpiochip caused the driver to request the wrong chip's line for
        # any pin not on that default chip, colliding with an unrelated,
        # already-claimed line (e.g. the kernel's own SPI0 hardware CS0)
        # and failing with a confusing "Device or resource busy" that had
        # nothing to do with the pin actually being fought over.
        def _chip_of(field):
            v = self.pin_lines.get(field)
            return self.gpiochip if v is None else v[0]
        self.pin_gpiochips = {
            "cs":    _chip_of("cs"),
            "irq":   _chip_of("irq"),
            "busy":  _chip_of("busy"),
            "reset": _chip_of("reset"),
            "txen":  _chip_of("txen"),
            "rxen":  _chip_of("rxen"),
        }

        # NOTICE log of the final resolved values — primary defense against
        # silent misconfiguration across the new 2-layer profile system.
        RNS.log(str(self) + " profile resolution:", RNS.LOG_NOTICE)
        RNS.log("  Platform  : " + str(self.platform_name)
                + " (v" + str(self.platform_version) + ")", RNS.LOG_VERBOSE)
        RNS.log("  Board     : " + str(self.board_name)
                + " (v" + str(self.board_version) + ")", RNS.LOG_VERBOSE)
        RNS.log("  gpiochip  : " + str(self.gpiochip), RNS.LOG_VERBOSE)
        RNS.log("  spi_bus   : " + str(self.spi_bus), RNS.LOG_VERBOSE)
        RNS.log("  spi_cs    : " + str(self.spi_cs), RNS.LOG_VERBOSE)
        for pin_field in _PIN_FIELDS:
            line = self.pin_lines.get(pin_field)
            if line is None:
                RNS.log("  pin_" + pin_field + "     : (not wired)", RNS.LOG_VERBOSE)
            else:
                RNS.log("  pin_" + pin_field + "     : " + str(line[0])
                        + " line " + str(line[1]), RNS.LOG_VERBOSE)
        RNS.log("  dio2_rf_switch    : " + str(self.dio2_rf_switch), RNS.LOG_VERBOSE)
        RNS.log("  dio3_tcxo_voltage : " + str(self.dio3_tcxo_voltage) + " V", RNS.LOG_VERBOSE)
        RNS.log("  tcxo_delay_ms     : " + str(self.tcxo_delay_ms), RNS.LOG_VERBOSE)
        RNS.log("  txpower_max       : " + str(self.txpower_max) + " dBm", RNS.LOG_VERBOSE)
        RNS.log("  rx_boosted_gain   : " + str(self.rx_boosted_gain), RNS.LOG_VERBOSE)

        # Log any per-key overrides the user specified
        for key, profile_val, override_val in resolution.get("overrides_used", []):
            RNS.log(str(self) + " override: " + key + " = " + str(override_val)
                    + " (profile default was " + str(profile_val) + ")",
                    RNS.LOG_WARNING)

        # ------------------------------------------------------------------
        # LoRa radio parameters (frequency / bandwidth / power / SF / CR)
        # ------------------------------------------------------------------
        frequency = int(c["frequency"]) if "frequency" in c else 915000000
        bandwidth = int(c["bandwidth"]) if "bandwidth" in c else 125000
        txpower   = int(c["txpower"])   if "txpower"   in c else 22
        sf        = int(c["spreadingfactor"]) if "spreadingfactor" in c else 8
        cr        = int(c["codingrate"]) if "codingrate" in c else 5

        # DIO3 TCXO voltage — for backward compat we also accept the legacy
        # `dio3_tcxo_voltage` config key and let it override the profile value.
        self.dio3_tcxo = self.dio3_tcxo_voltage
        if "dio3_tcxo_voltage" in c:
            self.dio3_tcxo = float(c["dio3_tcxo_voltage"])

        # Sync word: 0x12 = private/RNode, 0x34 = public/LoRaWAN.
        if "sync_word" in c:
            sw = c["sync_word"].strip()
            sync_word = int(sw, 16) if sw.startswith("0x") or sw.startswith("0X") else int(sw)
        else:
            sync_word = 0x12

        # Preamble length (LoRa preamble symbol count).
        #
        # IMPORTANT: mainline RNode_Firmware does NOT use a fixed preamble of
        # 8 symbols (a previous version of this comment incorrectly assumed
        # that). RNode dynamically computes its preamble length in
        # updateBitrate() (Utilities.h) to target ~24ms of preamble airtime
        # (LORA_PREAMBLE_TARGET_MS=24, reduced by LORA_PREAMBLE_FAST_DELTA=18
        # when bitrate > LORA_FAST_THRESHOLD_BPS=30000bps), with a hard floor
        # of LORA_PREAMBLE_SYMBOLS_MIN=18 symbols. At SF7/BW125000/CR5 (the
        # common default) this computes to 24 symbols, NOT 8. An 8-symbol
        # preamble is too short for a stock RNode receiver's correlator to
        # reliably lock onto, causing silent one-way RX failure in the
        # RNode-receives-from-us direction (TX LED lights, frame radiates,
        # but the peer's demodulator never detects it) while still allowing
        # this driver to receive RNode's own (longer) preambles fine — an
        # asymmetric failure that looks like a hardware fault but isn't.
        #
        # We replicate RNode's exact auto-tune formula by default so this
        # driver interoperates with stock RNode nodes out of the box. An
        # explicit `preamble_length` config value always overrides this.
        if "preamble_length" in c:
            try:
                _pl = int(c["preamble_length"])
                preamble_length = _pl if _pl > 0 else _rnode_preamble_symbols(sf, bandwidth, cr)
            except (ValueError, TypeError):
                preamble_length = _rnode_preamble_symbols(sf, bandwidth, cr)
        else:
            preamble_length = _rnode_preamble_symbols(sf, bandwidth, cr)

        # CSMA/CA parameters
        self.csma_p           = float(c["csma_p"])         if "csma_p"         in c else 0.1
        self.csma_slot_ms     = float(c["csma_slot_ms"])   if "csma_slot_ms"   in c else 50.0
        self.csma_max_backoff = int(c["csma_max_backoff"]) if "csma_max_backoff" in c else 5

        # Airtime limits (optional, percent 0-100)
        self.st_alock = float(c["airtime_limit_short"]) if "airtime_limit_short" in c else None
        self.lt_alock = float(c["airtime_limit_long"])  if "airtime_limit_long"  in c else None

        # ---- Instance state ----
        # The Interface base class sets self.HW_MTU = None in __init__; we
        # override that here with the value that matches RNode's wire format.
        self.HW_MTU    = SX126xInterface.HW_MTU
        self.owner     = owner
        self.name      = name
        self.online    = False
        self.detached  = False
        self.IN        = True
        self.OUT       = True

        self.frequency = frequency
        self.bandwidth = bandwidth
        self.txpower   = txpower
        self.sf        = sf
        self.cr        = cr
        self.sync_word = sync_word
        self.preamble_length = preamble_length

        self.bitrate     = 0
        self.r_stat_rssi = None
        self.r_stat_snr  = None
        self.announce_rate_target = None

        # Radio hardware handle (set by _init_radio)
        self.radio = None

        # Threading primitives
        self._stop_event   = None   # threading.Event set by detach()
        self._radio_thread = None   # the single radio-owner thread
        self._tx_queue     = None   # queue.Queue for outbound raw frames

        # Sliding-window airtime accounting. Each entry is (monotonic_ts, toa).
        self._airtime_short_deque = deque()
        self._airtime_long_deque  = deque()

        # RX fragment reassembly state (touched only from the radio thread)
        self._rx_fragments = {}
        self._frag_timeout = 10.0

        # Lockup-recovery counters (touched only from the radio thread)
        self._consecutive_spi_failures = 0
        self._consecutive_reinit_failures = 0

        # Validate configuration
        validcfg = True
        if frequency < SX126xInterface.FREQ_MIN or frequency > SX126xInterface.FREQ_MAX:
            RNS.log("Invalid frequency configured for " + str(self), RNS.LOG_ERROR)
            validcfg = False
        if txpower < -9 or txpower > 22:
            RNS.log("Invalid TX power configured for " + str(self), RNS.LOG_ERROR)
            validcfg = False
        # Enforce the resolved board profile's txpower_max as a hard ceiling,
        # not just an informational value (previously only logged, never
        # actually checked against the user-configured txpower - a real gap,
        # since some board profiles have external PAs where the chip-level
        # dBm setting does NOT correspond 1:1 to actual radiated power, and
        # exceeding a board's safe chip-level ceiling can mean exceeding the
        # board's actual safe RF output limit). Clamp rather than reject, so
        # a config written for one board (or with a stale/high txpower left
        # over from testing) doesn't take the whole interface offline - just
        # silently caps to what's safe for this specific board.
        if txpower > self.txpower_max:
            RNS.log(str(self) + " configured txpower " + str(txpower) +
                    " dBm exceeds this board's safe txpower_max of " +
                    str(self.txpower_max) + " dBm - capping to " +
                    str(self.txpower_max) + " dBm", RNS.LOG_WARNING)
            txpower = self.txpower_max
            self.txpower = txpower
        if bandwidth not in [7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000]:
            RNS.log("Invalid bandwidth configured for " + str(self), RNS.LOG_ERROR)
            validcfg = False
        if sf < 5 or sf > 12:
            RNS.log("Invalid spreading factor configured for " + str(self), RNS.LOG_ERROR)
            validcfg = False
        if cr < 5 or cr > 8:
            RNS.log("Invalid coding rate configured for " + str(self), RNS.LOG_ERROR)
            validcfg = False

        if not validcfg:
            raise ValueError("The configuration for " + str(self) + " contains errors, interface is offline")

        try:
            self._init_radio()
            self._update_bitrate()

            # Bring up the radio-owner thread and its queue
            self._stop_event = threading.Event()
            self._tx_queue   = queue.Queue(maxsize=SX126xInterface.TX_QUEUE_MAX)
            self._radio_thread = threading.Thread(
                target=self._radio_thread_main,
                name="SX126x[" + self.name + "]-radio",
                daemon=True,
            )
            self._radio_thread.start()

            self.online = True
            RNS.log(str(self) + " is now online", RNS.LOG_NOTICE)
            RNS.log("  Frequency : " + str(self.frequency / 1e6) + " MHz", RNS.LOG_VERBOSE)
            RNS.log("  Bandwidth : " + str(self.bandwidth / 1e3) + " kHz", RNS.LOG_VERBOSE)
            RNS.log("  TX Power  : " + str(self.txpower) + " dBm", RNS.LOG_VERBOSE)
            RNS.log("  SF        : " + str(self.sf), RNS.LOG_VERBOSE)
            RNS.log("  CR        : 4/" + str(self.cr), RNS.LOG_VERBOSE)
            RNS.log("  On-air    : " + str(round(self.bitrate / 1000.0, 2)) + " kbps", RNS.LOG_VERBOSE)

        except Exception as e:
            RNS.log("Could not initialise SX126x radio for " + str(self) + ": " + str(e), RNS.LOG_ERROR)
            # Best-effort cleanup of any partial state
            if self.radio is not None:
                try:
                    self.radio.close()
                except Exception:
                    pass
                self.radio = None
            raise

    # -----------------------------------------------------------------
    # Radio bring-up
    # -----------------------------------------------------------------

    def _init_radio(self):
        """Open the SX126x radio via the vendored driver and configure it."""
        vd = self.vd

        RNS.log(str(self) + " Initialising SX126x on SPI bus " + str(self.spi_bus) +
                " CS " + str(self.spi_cs), RNS.LOG_VERBOSE)

        self.radio = vd.SX126xRadio(
            spi_bus=self.spi_bus,
            spi_cs=self.spi_cs,
            spi_speed=2_000_000,
            pin_reset=self.pin_reset,
            pin_busy=self.pin_busy,
            pin_irq=self.pin_irq,
            pin_txen=self.pin_txen,
            pin_rxen=self.pin_rxen,
            txen_active_low=self.txen_active_low,
            rxen_active_low=self.rxen_active_low,
            pin_cs=self.pin_cs,
            gpiochip=self.gpiochip,
            pin_gpiochips=self.pin_gpiochips,
            dio3_tcxo_voltage=self.dio3_tcxo if self.dio3_tcxo and self.dio3_tcxo > 0 else None,
            dio3_tcxo_delay_ms=self.tcxo_delay_ms,
            busy_timeout_ms=5000,
        )

        self.radio.open()

        # DIO2 as RF switch control (for E22-style modules). Honour the
        # profile's dio2_rf_switch setting.
        if self.dio2_rf_switch:
            self.radio.set_dio2_as_rf_switch_ctrl(True)

        # DC-DC regulator for better efficiency
        self.radio.set_regulator_mode(vd.REGULATOR_DC_DC)

        # Carrier frequency
        self.radio.set_frequency(self.frequency)

        # TX power (SX1262 variant)
        self.radio.set_tx_power(self.txpower, vd.TX_POWER_SX1262)

        # RX gain: profile-controlled. Default is boosted for best sensitivity.
        if self.rx_boosted_gain:
            self.radio.set_rx_gain(vd.RX_GAIN_BOOSTED)
        else:
            self.radio.set_rx_gain(vd.RX_GAIN_POWER_SAVING)

        # Modulation
        ldro = self._should_use_ldro()
        self.radio.set_lora_modulation(self.sf, self.bandwidth, self.cr, ldro)

        # Packet params: explicit header, configurable preamble length
        # (default 8 symbols, override via `preamble_length` config key to
        # interoperate with firmwares that use a non-default preamble such
        # as 16), max 255-byte payload, CRC on, no IQ invert.
        self.radio.set_lora_packet(
            vd.HEADER_EXPLICIT,
            self.preamble_length,
            SX126xInterface.LORA_MAX_PAYLOAD,
            True,   # CRC enabled
            False,  # IQ not inverted
        )

        # Sync word
        self.radio.set_sync_word(self.sync_word)

        # CAD parameters for channel-activity-detection-based carrier sense.
        # The chip's CAD takes cad_symbol_num * t_sym time, so we pick a
        # conservative 8-symbol window with reasonable detection thresholds.
        self.radio.set_cad_params(
            cad_symbol_num=0x08,    # 8 symbols
            cad_det_peak=0x14,     # peak threshold (chip-internal)
            cad_det_min=0x0A,      # min threshold (chip-internal)
            cad_exit_mode=0x00,    # only exit on CAD done
            cad_timeout=0x008000,   # ~32ms chip-internal timeout
        )

        # Arm the IRQ line as a kernel edge-event so wait_irq_done() can
        # block on it instead of polling. The driver falls back to a 5ms
        # poll if the IRQ pin isn't wired or this fails for any reason.
        self.radio.request_irq_edge("rising")

        RNS.log(str(self) + " SX126x radio initialised successfully", RNS.LOG_VERBOSE)

    def _attempt_reinit(self):
        """Reset and reinitialise the chip after a lockup. SPI/GPIO handles
        stay open; only the chip state needs to be re-established.

        Returns True on success, False on any failure."""
        vd = self.vd
        try:
            try:
                self.radio.reset()
            except Exception as e:
                RNS.log(str(self) + " radio.reset() raised: " + str(e), RNS.LOG_WARNING)

            # Bring the chip back to a known good state. If any single step
            # raises, this function returns False (caller escalates).
            self.radio.set_standby(vd.STANDBY_RC)
            if self.radio.get_status_and_mode() != vd.STATUS_MODE_STDBY_RC:
                raise Exception("chip not responding to GetStatus after reset")

            self.radio.set_packet_type(vd.PACKET_TYPE_LORA)

            if self.dio3_tcxo and self.dio3_tcxo > 0:
                voltage = vd.SX126xRadio._tcxo_voltage_code(self.dio3_tcxo)
                delay   = vd.SX126xRadio._tcxo_delay_code(5.0)
                self.radio.set_dio3_as_tcxo_ctrl(voltage, delay)
                self.radio.set_standby(vd.STANDBY_RC)
                self.radio.calibrate(0xFF)
                # XOSC_START_ERR latches in the device-errors register on
                # every cold start on TCXO boards (datasheet §13.5.13).
                # Clear it so future diagnostics aren't confused by a stale
                # latched error from this reinit.
                try:
                    self.radio.clear_device_errors()
                except Exception:
                    pass

            self.radio.set_dio2_as_rf_switch_ctrl(self.dio2_rf_switch)
            self.radio.set_regulator_mode(vd.REGULATOR_DC_DC)
            self.radio.set_frequency(self.frequency)
            self.radio.set_tx_power(self.txpower, vd.TX_POWER_SX1262)
            if self.rx_boosted_gain:
                self.radio.set_rx_gain(vd.RX_GAIN_BOOSTED)
            else:
                self.radio.set_rx_gain(vd.RX_GAIN_POWER_SAVING)

            ldro = self._should_use_ldro()
            self.radio.set_lora_modulation(self.sf, self.bandwidth, self.cr, ldro)

            self.radio.set_lora_packet(
                vd.HEADER_EXPLICIT,
                self.preamble_length,
                SX126xInterface.LORA_MAX_PAYLOAD,
                True,
                False,
            )

            self.radio.set_sync_word(self.sync_word)

            self.radio.set_cad_params(
                cad_symbol_num=0x08,
                cad_det_peak=0x14,
                cad_det_min=0x0A,
                cad_exit_mode=0x00,
                cad_timeout=0x008000,
            )

            # Re-arm the IRQ edge subscription in case the line was disturbed
            try:
                self.radio.request_irq_edge("rising")
            except Exception:
                pass

            # Re-enter RX
            self._enter_rx_mode()
            return True

        except Exception as e:
            RNS.log(str(self) + " reinit failed: " + str(e), RNS.LOG_WARNING)
            return False

    # -----------------------------------------------------------------
    # Radio params + airtime math (pure functions, no radio I/O)
    # -----------------------------------------------------------------

    def _should_use_ldro(self):
        """LDRO (Low Data Rate Optimisation) is required when symbol time > 16 ms."""
        symbol_time = (2 ** self.sf) / self.bandwidth
        return symbol_time > 0.016

    def _update_bitrate(self):
        """Calculate on-air bitrate using the same formula RNode uses."""
        try:
            self.bitrate = self.sf * ((4.0 / self.cr) / (math.pow(2, self.sf) / (self.bandwidth / 1000.0))) * 1000
        except Exception:
            self.bitrate = 0

    def _calculate_toa(self, payload_len):
        """Time-on-air in seconds for a LoRa frame of `payload_len` bytes."""
        bw = self.bandwidth
        sf = self.sf
        cr = self.cr
        # Use the configured preamble length so airtime budgets are accurate
        # when interoperating with firmwares that use a non-default preamble
        # (e.g. 16). A mismatch here would make the CSMA / airtime-limiter
        # under-count real on-air time and silently exceed the configured
        # duty cycle.
        preamble_len = self.preamble_length
        has_crc = True
        explicit_header = True
        ldro = self._should_use_ldro()

        t_sym = (2 ** sf) / bw
        t_preamble = (preamble_len + 4.25) * t_sym

        de = 1 if ldro else 0
        ih = 0 if explicit_header else 1
        crc_bits = 16 if has_crc else 0

        num = max(8 * payload_len - 4 * sf + 28 + crc_bits - 20 * ih, 0)
        denom = 4 * (sf - 2 * de)
        n_payload = 8 + math.ceil(num / denom) * cr

        return t_preamble + n_payload * t_sym

    # -----------------------------------------------------------------
    # Split-frame framing
    # -----------------------------------------------------------------

    def _make_frames(self, data):
        """Split data into LoRa frames with RNode-compatible framing.

        Each frame gets a 1-byte header:
          - Upper nibble: 4-bit random sequence ID
          - Bit 0 (FLAG_SPLIT): 1 if packet is split across 2 frames
        Packets <= 254 bytes: single frame, FLAG_SPLIT=0
        Packets >  254 bytes: split into 2 frames, FLAG_SPLIT=1
        """
        max_payload = SX126xInterface.FRAME_PAYLOAD_MAX
        frames = []

        if len(data) <= max_payload:
            seq_id = random.randint(0, 15) << 4
            header = seq_id & 0xF0  # FLAG_SPLIT = 0
            frames.append(bytes([header]) + data)
        else:
            seq_id = random.randint(0, 15) << 4
            header = (seq_id & 0xF0) | SX126xInterface.FLAG_SPLIT
            split_point = max_payload
            frames.append(bytes([header]) + data[:split_point])
            frames.append(bytes([header]) + data[split_point:])

        return frames

    def _reassemble(self, frame_data):
        """Reassemble a received frame using the split-packet protocol.

        Returns the reassembled packet, or None if waiting for the second
        fragment. Stale fragments (> self._frag_timeout seconds old) are
        discarded silently.
        """
        if len(frame_data) < 1:
            return None

        header = frame_data[0]
        seq_id = header & 0xF0
        is_split = bool(header & SX126xInterface.FLAG_SPLIT)
        payload = bytes(frame_data[1:])

        if not is_split:
            return payload

        now = time.monotonic()
        # Clean stale fragments
        stale = [k for k, (ts, _) in self._rx_fragments.items()
                 if now - ts > self._frag_timeout]
        for k in stale:
            del self._rx_fragments[k]

        if seq_id in self._rx_fragments:
            _, first_payload = self._rx_fragments.pop(seq_id)
            return first_payload + payload
        else:
            self._rx_fragments[seq_id] = (now, payload)
            return None

    # -----------------------------------------------------------------
    # Reticulum interface contract
    # -----------------------------------------------------------------

    def should_ingress_limit(self):
        # LoRa interfaces don't apply ingress limiting at the interface layer;
        # the radio itself is the bottleneck.
        return False

    def process_incoming(self, data):
        """Called by the radio thread when a fully reassembled packet is RX'd."""
        self.rxb += len(data)
        self.owner.inbound(data, self)
        # Reset per-packet stats so a stale RSSI/SNR doesn't bleed into the
        # next packet's signalling (matches RNode behaviour).
        self.r_stat_rssi = None
        self.r_stat_snr  = None

    def process_outgoing(self, data):
        """Called by Reticulum (Transport / announce thread) to send a packet.

        MUST be non-blocking: push onto the queue and return. The radio-owner
        thread is the only thread that ever touches the radio.
        """
        if not self.online or self._stop_event is None:
            return
        if self._tx_queue is None:
            return

        datalen = len(data)
        if datalen > self.HW_MTU:
            RNS.log(str(self) + " dropping oversized packet ("
                    + str(datalen) + " > " + str(self.HW_MTU) + ")", RNS.LOG_ERROR)
            return

        try:
            self._tx_queue.put_nowait(bytes(data))
        except queue.Full:
            RNS.log(str(self) + " TX queue full, dropping " + str(datalen)
                    + "-byte packet", RNS.LOG_WARNING)

    # -----------------------------------------------------------------
    # Radio-owner thread (the ONLY thread that touches the radio)
    # -----------------------------------------------------------------

    def _radio_thread_main(self):
        """Main loop of the single radio-owner thread."""
        RNS.log(str(self) + " radio thread started", RNS.LOG_VERBOSE)

        try:
            self._enter_rx_mode()
        except Exception as e:
            RNS.log(str(self) + " failed to enter initial RX: " + str(e), RNS.LOG_ERROR)
            self.online = False
            return

        # Diagnostic heartbeat: periodically log the current IRQ status even
        # if no IRQ fired, so we can confirm the RX path is alive (the chip
        # is in RX mode, IRQs are armed) and detect any silent state changes.
        import time as _diag_time
        _last_heartbeat = _diag_time.monotonic()

        while not self._stop_event.is_set():
            # ---- 1. Block briefly on the radio's IRQ edge ----
            irq = None
            try:
                irq = self.radio.wait_irq_done(0.1)
                self._consecutive_spi_failures = 0
            except Exception as e:
                RNS.log(str(self) + " wait_irq_done exception: " + str(e), RNS.LOG_WARNING)
                if not self._handle_spi_failure("wait_irq_done", e):
                    return  # interface marked offline

            # ---- 2. Dispatch any IRQ that fired ----
            if irq is not None and irq != 0:
                try:
                    self._handle_irq(irq)
                except Exception as e:
                    if not self._handle_spi_failure("handle_irq", e):
                        return

            # ---- 3. Drain one TX-queue entry if budget + CAD allow ----
            try:
                self._drain_tx_queue()
            except Exception as e:
                if not self._handle_spi_failure("drain_tx_queue", e):
                    return

            # ---- 4. Diagnostic heartbeat (every 5s) ----
            now = _diag_time.monotonic()
            if now - _last_heartbeat >= 5.0:
                _last_heartbeat = now
                try:
                    cur_irq = self.radio.get_irq_status()
                    cur_status = self.radio.get_status_byte()
                    RNS.log(
                        str(self) + " heartbeat: status=0x{:02x} irq=0x{:04x} "
                        "(queue size={})".format(
                            cur_status, cur_irq,
                            self._tx_queue.qsize() if self._tx_queue is not None else -1,
                        ),
                        RNS.LOG_INFO,
                    )
                except Exception as e:
                    RNS.log(str(self) + " heartbeat SPI failure: " + str(e), RNS.LOG_WARNING)
                    if not self._handle_spi_failure("heartbeat", e):
                        return

        RNS.log(str(self) + " radio thread exiting", RNS.LOG_VERBOSE)

    def _handle_irq(self, irq):
        """Dispatch an SX126x IRQ status word to the appropriate handler."""
        vd = self.vd
        # Log every IRQ activity at LOG_INFO so RX-side noise is visible
        # during diagnostics (any received RF energy that fails to decode as
        # a valid LoRa packet shows up as IRQ_HEADER_ERR / IRQ_CRC_ERR).
        RNS.log(
            str(self) + " IRQ status=0x{:04x} (RX_DONE={} TX_DONE={} "
            "TIMEOUT={} CRC_ERR={} HEADER_ERR={} PREAMBLE={} SYNC={} "
            "CAD_DONE={} CAD_DETECTED={})".format(
                irq,
                bool(irq & vd.IRQ_RX_DONE),
                bool(irq & vd.IRQ_TX_DONE),
                bool(irq & vd.IRQ_TIMEOUT),
                bool(irq & vd.IRQ_CRC_ERR),
                bool(irq & vd.IRQ_HEADER_ERR),
                bool(irq & vd.IRQ_PREAMBLE_DETECTED),
                bool(irq & vd.IRQ_SYNC_WORD_VALID),
                bool(irq & vd.IRQ_CAD_DONE),
                bool(irq & vd.IRQ_CAD_DETECTED),
            ),
            RNS.LOG_INFO,
        )
        if irq & vd.IRQ_RX_DONE:
            self._handle_rx_done()
        if irq & vd.IRQ_TX_DONE:
            self._handle_tx_done()
        if irq & vd.IRQ_TIMEOUT:
            RNS.log(str(self) + " radio reported timeout IRQ", RNS.LOG_INFO)
            try:
                self.radio.clear_irq_status(vd.IRQ_TIMEOUT)
            except Exception:
                pass
        if irq & vd.IRQ_CRC_ERR:
            RNS.log(str(self) + " CRC error on received frame", RNS.LOG_INFO)
            try:
                self.radio.clear_irq_status(vd.IRQ_CRC_ERR)
            except Exception:
                pass
        if irq & vd.IRQ_HEADER_ERR:
            RNS.log(str(self) + " header error on received frame (likely sync-word/header-mode mismatch)", RNS.LOG_INFO)
            try:
                self.radio.clear_irq_status(vd.IRQ_HEADER_ERR)
            except Exception:
                pass
        if irq & vd.IRQ_PREAMBLE_DETECTED:
            RNS.log(str(self) + " preamble detected (RF energy is arriving)", RNS.LOG_INFO)
            try:
                self.radio.clear_irq_status(vd.IRQ_PREAMBLE_DETECTED)
            except Exception:
                pass
        if irq & vd.IRQ_SYNC_WORD_VALID:
            RNS.log(str(self) + " sync word valid (LoRa preamble+sync matched)", RNS.LOG_INFO)
            try:
                self.radio.clear_irq_status(vd.IRQ_SYNC_WORD_VALID)
            except Exception:
                pass
        # CAD IRQs are handled inline inside _channel_is_clear(); clear them
        # defensively in case a CAD IRQ leaked through.
        if irq & (vd.IRQ_CAD_DONE | vd.IRQ_CAD_DETECTED):
            try:
                self.radio.clear_irq_status(vd.IRQ_CAD_DONE | vd.IRQ_CAD_DETECTED)
            except Exception:
                pass

    def _handle_rx_done(self):
        """Read the just-received frame out of the chip's buffer and pass it
        on to the Reticulum stack (via _reassemble + process_incoming)."""
        vd = self.vd
        try:
            payload_len, buf_offset = self.radio.get_rx_buffer_status()
            if payload_len <= 0:
                # IRQ_RX_DONE fired but nothing in the buffer — probably a
                # header/CRC error swallowed the frame. Clear IRQs and exit.
                try:
                    status_byte = self.radio.get_status_byte()
                    irq_now = self.radio.get_irq_status()
                    RNS.log(
                        str(self) + " payload_len<=0 on RX_DONE: "
                        "status=0x{:02x}".format(status_byte)
                        + " irq_now=0x{:04x}".format(irq_now),
                        RNS.LOG_DEBUG,
                    )
                except Exception:
                    pass
                try:
                    self.radio.clear_irq_status(vd.IRQ_RX_DONE | vd.IRQ_CRC_ERR | vd.IRQ_HEADER_ERR)
                except Exception:
                    pass
                return

            rssi_raw, snr_raw, _sig_raw = self.radio.get_packet_status()
            frame_data = bytes(self.radio.read_buffer(buf_offset, payload_len))

            # Per datasheet: rssi_dbm = raw / -2, snr_db = (raw if <128 else raw-256) / 4
            rssi_dbm = rssi_raw / -2.0
            snr_db = (snr_raw if snr_raw < 128 else snr_raw - 256) / 4.0

            self.r_stat_rssi = rssi_dbm
            self.r_stat_snr  = snr_db

            RNS.log(str(self) + " received frame ("
                    + str(len(frame_data)) + " bytes, RSSI: "
                    + f"{rssi_dbm:.1f}" + ", SNR: " + f"{snr_db:.1f}"
                    + ")", RNS.LOG_DEBUG)

            # Log the raw first ~16 bytes of the RX buffer at DEBUG level.
            # Useful for confirming frame/destination-hash alignment when
            # diagnosing interop with other Reticulum-compatible firmwares.
            preview = frame_data[:16]
            RNS.log(
                str(self) + " rx-preview len=" + str(len(frame_data))
                + " head=" + preview.hex() + " rssi=" + f"{rssi_dbm:.1f}"
                + " snr=" + f"{snr_db:.1f}",
                RNS.LOG_DEBUG,
            )

            packet = self._reassemble(frame_data)
            if packet is not None:
                self.process_incoming(packet)
        except Exception as e:
            RNS.log(str(self) + " error handling RX_DONE: " + str(e), RNS.LOG_WARNING)
        finally:
            try:
                self.radio.clear_irq_status(vd.IRQ_RX_DONE | vd.IRQ_CRC_ERR | vd.IRQ_HEADER_ERR)
            except Exception:
                pass

    def _handle_tx_done(self):
        """Acknowledge IRQ_TX_DONE. The actual frame accounting happens in
        _drain_tx_queue, which is where we know which frame just finished."""
        try:
            self.radio.clear_irq_status(self.vd.IRQ_TX_DONE)
        except Exception:
            pass
        # Return to RX. The IRQ for this transition is harmless (the line
        # will pulse as we change modes, but we'll re-enter RX and clear
        # status again).
        try:
            self._enter_rx_mode()
        except Exception:
            pass

    def _enter_rx_mode(self):
        """Put the chip into continuous receive mode and arm the IRQ mask
        for RX events. Safe to call repeatedly."""
        vd = self.vd
        irq_mask = vd.IRQ_RX_DONE | vd.IRQ_TIMEOUT | vd.IRQ_CRC_ERR | vd.IRQ_HEADER_ERR
        # Drive the board's external RXEN/TXEN GPIOs (if any) so the E22
        # module's RX path is enabled. No-op when the board doesn't define
        # them.
        self.radio.set_rx_enable(True)
        # Explicitly restore the LoRa packet params to the maximum payload
        # length (255) before issuing SetRx. The previous SetPacketParams
        # call (from the most recent _transmit_frame_blocking) would have
        # left payloadLength at len(frame) — which is fine for the receiver
        # to accept packets up to that size, but it's much more useful to
        # have the RX path accept the full 255-byte maximum rather than
        # whatever length the most recent TX happened to be. Explicit is
        # also defensive: if RX mode is entered before any TX (e.g. on
        # initial boot), we don't rely on the chip's default being 255.
        self.radio.set_lora_packet(
            vd.HEADER_EXPLICIT,
            self.preamble_length,
            SX126xInterface.LORA_MAX_PAYLOAD,
            True,
            False,
        )
        self.radio.set_standby(vd.STANDBY_RC)
        self.radio.set_dio_irq_params(irq_mask, dio1_mask=irq_mask)
        self.radio.clear_irq_status(vd.IRQ_ALL)
        self.radio.set_rx(vd.RX_CONTINUOUS)

    # -----------------------------------------------------------------
    # Carrier sense (CAD)
    # -----------------------------------------------------------------

    def _channel_is_clear(self):
        """CAD-based carrier sense.

        Returns True if the channel appears clear (no preamble detected
        during an 8-symbol CAD window), False if a preamble was detected
        or the CAD operation failed/timed out. Always returns the chip to
        RX mode before returning (the chip drops back to STDBY after CAD).

        CAD parameters are configured once in _init_radio().
        """
        vd = self.vd
        try:
            self.radio.set_standby(vd.STANDBY_RC)
            cad_mask = vd.IRQ_CAD_DONE | vd.IRQ_CAD_DETECTED
            self.radio.set_dio_irq_params(cad_mask, dio1_mask=cad_mask)
            self.radio.clear_irq_status(vd.IRQ_ALL)
            self.radio.set_cad()

            # 8 CAD symbols at SF8/BW125k ≈ 16ms; 50ms is generous.
            # At SF12/BW125k it's ≈130ms, so on slow configs the CAD itself
            # may exceed this budget — treat that as "busy" (return False)
            # which is conservative.
            irq = self.radio.wait_irq_done(0.05)

            try:
                self.radio.clear_irq_status(cad_mask)
            except Exception:
                pass

            # Return to RX regardless of outcome so the next IRQ wait picks
            # up incoming frames.
            self._enter_rx_mode()

            if irq is None:
                return False
            return (irq & vd.IRQ_CAD_DETECTED) == 0

        except Exception as e:
            # On error, try to get back to RX and be permissive (return
            # True) — being slightly too eager to TX is better than
            # refusing to TX at all when the chip is glitching.
            try:
                self._enter_rx_mode()
            except Exception:
                pass
            RNS.log(str(self) + " CAD error: " + str(e), RNS.LOG_DEBUG)
            return True

    def _csma_wait(self):
        """P-persistent CSMA/CA with CAD-based carrier sense.

        Returns True if the channel is clear and we should proceed to TX,
        False if we exhausted our backoff attempts (channel busy)."""
        slot_time = self.csma_slot_ms / 1000.0
        max_attempts = self.csma_max_backoff * 10

        for attempt in range(max_attempts):
            if self._stop_event.is_set():
                return False
            if self._channel_is_clear():
                if random.random() < self.csma_p:
                    return True
                time.sleep(slot_time)
            else:
                backoff_exp = min(attempt // 2, self.csma_max_backoff)
                max_slots = 2 ** backoff_exp
                wait_slots = random.randint(0, max_slots)
                time.sleep(wait_slots * slot_time)

        return False

    # -----------------------------------------------------------------
    # TX path
    # -----------------------------------------------------------------

    def _drain_tx_queue(self):
        """Take one packet off the TX queue and try to transmit it.

        Pre-flight checks (in order):
          1. Compute total TOA across all split-frame fragments.
          2. Check sliding-window airtime budget — drop if exceeded.
          3. Run CSMA — drop if exhausted without finding the channel clear.
        """
        if self._tx_queue is None or self._tx_queue.empty():
            return

        try:
            data = self._tx_queue.get_nowait()
        except queue.Empty:
            return

        frames = self._make_frames(data)
        total_toa = sum(self._calculate_toa(len(f)) for f in frames)

        # Airtime pre-check (drop before consuming any airtime budget)
        if not self._airtime_budget_allows(total_toa):
            cur_short = self._get_short_airtime_pct()
            add_short = (total_toa / SX126xInterface.AIRTIME_SHORT_WINDOW) * 100.0
            cur_long = self._get_long_airtime_pct()
            add_long = (total_toa / SX126xInterface.AIRTIME_LONG_WINDOW) * 100.0
            RNS.log(str(self) + " dropping packet: airtime budget would be "
                    "exceeded (short: " + f"{cur_short:.1f}" + "% + "
                    + f"{add_short:.1f}" + "%, long: " + f"{cur_long:.1f}"
                    + "% + " + f"{add_long:.1f}" + "%)",
                    RNS.LOG_WARNING)
            return

        # CSMA
        if not self._csma_wait():
            RNS.log(str(self) + " dropping packet: CSMA gave up after "
                    + str(self.csma_max_backoff * 10) + " attempts (channel busy)",
                    RNS.LOG_WARNING)
            return

        # Transmit each fragment
        for i, frame in enumerate(frames):
            fragment_toa = self._calculate_toa(len(frame))
            try:
                ok = self._transmit_frame_blocking(frame)
            except Exception as e:
                if not self._handle_spi_failure("transmit_frame", e):
                    return
                ok = False

            if not ok:
                RNS.log(str(self) + " TX failed on fragment "
                        + str(i + 1) + "/" + str(len(frames)), RNS.LOG_WARNING)
                try:
                    self._enter_rx_mode()
                except Exception:
                    pass
                return

            # Record airtime only after a confirmed TX so we don't credit
            # failed transmissions.
            self._track_airtime(fragment_toa)

            if i < len(frames) - 1:
                time.sleep(0.005)  # Inter-frame gap

        self.txb += len(data)

        try:
            self._enter_rx_mode()
        except Exception as e:
            if not self._handle_spi_failure("re_enter_rx_after_tx", e):
                return

    def _transmit_frame_blocking(self, frame):
        """Transmit a single LoRa frame and block until TX done or timeout.

        Returns True on IRQ_TX_DONE, False on timeout / IRQ_TIMEOUT / error.
        On timeout we still try to put the chip back into standby so the
        next operation can recover."""
        vd = self.vd

        # Per-frame TX timeout: 1.5x the frame's TOA + 1s margin, clamped
        # to a sane range. This bounds shutdown latency while covering the
        # worst-case (SF=12, BW=125k, 254-byte payload ≈ 9s).
        toa = self._calculate_toa(len(frame))
        timeout_s = max(min(toa * 1.5 + 1.0, 15.0), 1.0)

        # Chip-internal TX timeout: 0 = "no chip-side timeout" (TX_SINGLE
        # convention from LoRaRF-Python). The chip will only signal TX_DONE
        # or, on a stuck-state chip, never signal at all (caught by the
        # host-side wait_irq_done above). We previously set a chip-side
        # timeout of 1.2x TOA here, but on some SX1262 firmware states a
        # rejected SetTx (status 0x2a) immediately fires IRQ_TIMEOUT
        # (~1ms), which is indistinguishable from a real TX timeout and
        # confuses the host. Letting the chip use TX_SINGLE and trusting
        # the host-side wait_irq_done is cleaner.
        chip_timeout_units = 0x000000

        try:
            self.radio.set_standby(vd.STANDBY_RC)
            # Drive the board's external TXEN/RXEN GPIOs (if any) so the
            # E22 module's PA path is enabled for TX. The vendored driver's
            # set_tx_enable() is a no-op when self._line_txen is None (i.e.
            # when the board profile set header_pin_txen = -1, as the
            # Femtofox-integrated profile does), so this is safe for both
            # "TXEN bridged to DIO2" and "TXEN on a separate GPIO" boards.
            self.radio.set_tx_enable(True)
            # Set the LoRa packet params with the EXACT payload length
            # we are about to transmit — per SX126x datasheet §13.4.6,
            # payloadLength is the number of bytes the modem transmits, NOT
            # a maximum. Using LORA_MAX_PAYLOAD here would cause the chip
            # to transmit len(frame) real bytes followed by (255-len(frame))
            # bytes of stale TX-buffer garbage, with the LoRa header
            # declaring length 255 and the CRC computed over all 255 bytes
            # — making the resulting RX_DONE clean (LoRa-level decode
            # succeeds) but the payload = real-packet-plus-garbage, which
            # Reticulum's higher-layer validation silently rejects.
            self.radio.set_lora_packet(
                vd.HEADER_EXPLICIT,
                self.preamble_length,
                len(frame),
                True,
                False,
            )
            self.radio.set_buffer_base_address(0, 0)
            self.radio.clear_irq_status(vd.IRQ_ALL)
            self.radio.set_dio_irq_params(
                vd.IRQ_TX_DONE | vd.IRQ_TIMEOUT,
                dio1_mask=vd.IRQ_TX_DONE | vd.IRQ_TIMEOUT,
            )
            self.radio.write_buffer(0, frame)
            self.radio.set_tx(chip_timeout_units)
            irq = self.radio.wait_irq_done(timeout_s)
            try:
                self.radio.clear_irq_status(vd.IRQ_TX_DONE | vd.IRQ_TIMEOUT)
            except Exception:
                pass
            # Restore TXEN/RXEN to their pre-TX state. No-op when the
            # board doesn't define them.
            self.radio.restore_tx_rx_pins()

            if irq is None:
                RNS.log(str(self) + " TX wait timed out", RNS.LOG_WARNING)
                return False
            if irq & vd.IRQ_TIMEOUT:
                RNS.log(str(self) + " TX reported timeout IRQ", RNS.LOG_WARNING)
                return False
            return bool(irq & vd.IRQ_TX_DONE)
        except Exception:
            # Bubble up to _drain_tx_queue, which will run it through
            # _handle_spi_failure.
            raise

    # -----------------------------------------------------------------
    # Airtime accounting (sliding-window deques)
    # -----------------------------------------------------------------

    def _track_airtime(self, toa_seconds):
        """Record a transmission's airtime into both sliding-window deques."""
        now = time.monotonic()
        short_cutoff = now - SX126xInterface.AIRTIME_SHORT_WINDOW
        long_cutoff  = now - SX126xInterface.AIRTIME_LONG_WINDOW

        while self._airtime_short_deque and self._airtime_short_deque[0][0] < short_cutoff:
            self._airtime_short_deque.popleft()
        while self._airtime_long_deque and self._airtime_long_deque[0][0] < long_cutoff:
            self._airtime_long_deque.popleft()

        self._airtime_short_deque.append((now, toa_seconds))
        self._airtime_long_deque.append((now, toa_seconds))

    def _get_short_airtime_pct(self):
        total = sum(toa for _, toa in self._airtime_short_deque)
        return (total / SX126xInterface.AIRTIME_SHORT_WINDOW) * 100.0

    def _get_long_airtime_pct(self):
        total = sum(toa for _, toa in self._airtime_long_deque)
        return (total / SX126xInterface.AIRTIME_LONG_WINDOW) * 100.0

    def _airtime_budget_allows(self, additional_toa):
        """True iff adding `additional_toa` would not exceed either configured
        airtime budget. Used as a pre-flight check before consuming any
        airtime for CSMA / TX."""
        if self.st_alock is not None:
            cur = self._get_short_airtime_pct()
            add = (additional_toa / SX126xInterface.AIRTIME_SHORT_WINDOW) * 100.0
            if cur + add > self.st_alock:
                return False
        if self.lt_alock is not None:
            cur = self._get_long_airtime_pct()
            add = (additional_toa / SX126xInterface.AIRTIME_LONG_WINDOW) * 100.0
            if cur + add > self.lt_alock:
                return False
        return True

    # -----------------------------------------------------------------
    # SPI failure tracking / radio reinit / offline
    # -----------------------------------------------------------------

    def _handle_spi_failure(self, op, exc):
        """Track consecutive SPI failures and escalate to reinit or offline.

        Returns True if the caller should continue the loop, False if the
        interface has been marked offline (in which case the radio thread
        should return).
        """
        self._consecutive_spi_failures += 1

        if self._consecutive_spi_failures < 3:
            RNS.log(str(self) + " SX126x SPI failure "
                    + str(self._consecutive_spi_failures) + " during "
                    + op + ": " + str(exc), RNS.LOG_WARNING)
            time.sleep(0.05)
            return True

        if self._consecutive_spi_failures < SX126xInterface.RADIO_REINIT_THRESHOLD:
            RNS.log(str(self) + " SX126x SPI failures persisting ("
                    + str(self._consecutive_spi_failures) + ") during "
                    + op + ": " + str(exc), RNS.LOG_WARNING)
            time.sleep(0.2)
            return True

        # Threshold reached — try a full radio reinit
        RNS.log(str(self) + " SX126x appears locked up ("
                + str(self._consecutive_spi_failures) + " SPI failures), "
                + "attempting reset + reinit", RNS.LOG_ERROR)

        if self._attempt_reinit():
            self._consecutive_spi_failures = 0
            self._consecutive_reinit_failures = 0
            RNS.log(str(self) + " SX126x reinit succeeded, resuming", RNS.LOG_NOTICE)
            return True

        self._consecutive_reinit_failures += 1
        if self._consecutive_reinit_failures >= SX126xInterface.RADIO_GIVEUP_THRESHOLD:
            RNS.log(str(self) + " SX126x reinit failed "
                    + str(self._consecutive_reinit_failures)
                    + " times in a row, marking interface offline", RNS.LOG_ERROR)
            self.online = False
            return False

        RNS.log(str(self) + " SX126x reinit failed ("
                + str(self._consecutive_reinit_failures)
                + "/" + str(SX126xInterface.RADIO_GIVEUP_THRESHOLD)
                + "), continuing", RNS.LOG_WARNING)
        time.sleep(0.5)
        return True

    # -----------------------------------------------------------------
    # Detach / shutdown
    # -----------------------------------------------------------------

    def detach(self):
        """Shut the interface down cleanly.

        Order of operations:
          1. Mark offline / detached so Reticulum stops calling process_outgoing.
          2. Signal the radio-owner thread to stop (Event).
          3. Join the radio thread with a bounded timeout.
          4. ONLY THEN close the radio (release SPI + GPIO handles).
        Closing radio resources while the thread might still be using them
        was a bug in the old code (now-fixed)."""
        self.detached = True
        self.online = False

        if self._stop_event is not None:
            self._stop_event.set()

        if self._radio_thread is not None:
            self._radio_thread.join(timeout=2.0)
            if self._radio_thread.is_alive():
                RNS.log(str(self) + " radio thread did not exit within 2s", RNS.LOG_WARNING)
            self._radio_thread = None

        if self.radio is not None:
            try:
                self.radio.close()
            except Exception as e:
                RNS.log(str(self) + " error during radio close: " + str(e), RNS.LOG_WARNING)
            self.radio = None

        RNS.log(str(self) + " detached", RNS.LOG_NOTICE)

    def __str__(self):
        return "SX126xInterface[" + self.name + "]"


# ---------------------------------------------------------------------------
# CRITICAL: Register the interface class so Reticulum's external-interface
# loader can find it. Without this line the module will load but the
# interface will not be instantiated.
# ---------------------------------------------------------------------------
interface_class = SX126xInterface