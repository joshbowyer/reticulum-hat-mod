##############################################################################
# vendored_sx126x.py                                                         #
#                                                                            #
# Minimal, single-purpose SX1261/SX1262/SX1268 (and LLCC68) driver on raw    #
# spidev + libgpiod 2.2.x (request-based Python API; migrated from the      #
# old 1.6.x Chip/Line API - see the GPIO helper methods for details).       #
#                                                                            #
# Replaces LoRaRF-Python (which busy-spins on wait() and burns 100% CPU).   #
# All SX126x command opcodes and parameter byte sequences were extracted     #
# from LoRaRF-Python's SX126x.py (Sep 2022, /tmp/opencode/lorarf/) as a     #
# ground-truth reference, rather than re-derived from the datasheet, so     #
# the byte-level command layout matches what the removed library was        #
# already successfully issuing to real RNode-interop hardware.             #
#                                                                            #
# Scope is intentionally narrow:                                             #
#   - SPI + GPIO plumbing                                                    #
#   - 1:1 SX126x command methods                                            #
#   - A few small composition helpers (frequency, tx power, modulation,     #
#     packet params, sync word) that the RNS-integration layer will need   #
#   - Blocking IRQ wait via libgpiod edge events (the entire reason this    #
#     file exists)                                                           #
#                                                                            #
# Deliberately NOT in this file (lives in SX126xInterface.py):              #
#   - Threading / queues / CSMA                                             #
#   - Split-packet framing / reassembly                                     #
#   - Reticulum Interface subclass                                          #
#   - Airtime accounting                                                     #
#                                                                            #
# License: MIT (matches the rest of this project)                           #
##############################################################################

import time
import threading

# These two are *only* imported at module import time on a real device
# that has them installed. On the development workstation they are not
# present, so we attempt the import lazily inside open()/close() to keep
# `python3 -m py_compile vendored_sx126x.py` working in CI/dev.
try:
    import spidev  # provided by Debian/Ubuntu package python3-spidev
except ImportError:
    spidev = None

try:
    import gpiod  # provided by Debian/Ubuntu package python3-libgpiod (v2.x)
    from gpiod.line import Direction as _GpiodDirection
    from gpiod.line import Value as _GpiodValue
    from gpiod.line import Edge as _GpiodEdge
except ImportError:
    gpiod = None
    _GpiodDirection = None
    _GpiodValue = None
    _GpiodEdge = None


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class SX126xError(IOError):
    """Base class for SX126x driver errors."""


class SX126xTimeout(SX126xError, TimeoutError):
    """Raised when a SPI transaction or IRQ wait exceeds its budget."""


# ---------------------------------------------------------------------------
# Constants (all opcodes + parameter values copied verbatim from
# /tmp/opencode/lorarf/LoRaRF/SX126x.py so the byte sequences sent on the
# wire are byte-identical to what the previously-working driver issued)
# ---------------------------------------------------------------------------

# ---- SX126x command opcodes (per datasheet §13, also confirmed by LoRaRF) ----
CMD_SET_SLEEP                  = 0x84
CMD_SET_STANDBY                = 0x80
CMD_SET_FS                     = 0xC1
CMD_SET_TX                     = 0x83
CMD_SET_RX                     = 0x82
CMD_SET_TIMER_ON_PREAMBLE      = 0x9F
CMD_SET_RX_DUTY_CYCLE          = 0x94
CMD_SET_CAD                    = 0xC5
CMD_SET_TX_CONTINUOUS_WAVE     = 0xD1
CMD_SET_TX_INFINITE_PREAMBLE   = 0xD2
CMD_SET_REGULATOR_MODE         = 0x96
CMD_CALIBRATE                  = 0x89
CMD_CALIBRATE_IMAGE            = 0x98
CMD_SET_PA_CONFIG              = 0x95
CMD_SET_RX_TX_FALLBACK_MODE    = 0x93
CMD_WRITE_REGISTER             = 0x0D
CMD_READ_REGISTER              = 0x1D
CMD_WRITE_BUFFER               = 0x0E
CMD_READ_BUFFER                = 0x1E
CMD_SET_DIO_IRQ_PARAMS         = 0x08
CMD_GET_IRQ_STATUS             = 0x12
CMD_CLEAR_IRQ_STATUS           = 0x02
CMD_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D
CMD_SET_DIO3_AS_TCXO_CTRL      = 0x97
CMD_SET_RF_FREQUENCY           = 0x86
CMD_SET_PACKET_TYPE            = 0x8A
CMD_GET_PACKET_TYPE            = 0x11
CMD_SET_TX_PARAMS              = 0x8E
CMD_SET_MODULATION_PARAMS      = 0x8B
CMD_SET_PACKET_PARAMS          = 0x8C
CMD_SET_CAD_PARAMS             = 0x88
CMD_SET_BUFFER_BASE_ADDRESS    = 0x8F
CMD_SET_LORA_SYMB_NUM_TIMEOUT  = 0xA0
CMD_GET_STATUS                 = 0xC0
CMD_GET_RX_BUFFER_STATUS       = 0x13
CMD_GET_PACKET_STATUS          = 0x14
CMD_GET_RSSI_INST              = 0x15
CMD_GET_STATS                  = 0x10
CMD_RESET_STATS                = 0x00
CMD_GET_DEVICE_ERRORS          = 0x17
CMD_CLEAR_DEVICE_ERRORS        = 0x07

# ---- Standby / sleep configs ----
STANDBY_RC             = 0x00
STANDBY_XOSC           = 0x01
SLEEP_COLD_START       = 0x00
SLEEP_WARM_START       = 0x04
SLEEP_COLD_START_RTC   = 0x01
SLEEP_WARM_START_RTC   = 0x05

# ---- Packet / modem type ----
PACKET_TYPE_GFSK       = 0x00
PACKET_TYPE_LORA       = 0x01

# ---- PA device selectors ----
TX_POWER_SX1261        = 0x01
TX_POWER_SX1262        = 0x02
TX_POWER_SX1268        = 0x08

# ---- DIO3 TCXO voltage selector ----
DIO3_OUTPUT_1_6        = 0x00
DIO3_OUTPUT_1_7        = 0x01
DIO3_OUTPUT_1_8        = 0x02
DIO3_OUTPUT_2_2        = 0x03
DIO3_OUTPUT_2_4        = 0x04
DIO3_OUTPUT_2_7        = 0x05
DIO3_OUTPUT_3_0        = 0x06
DIO3_OUTPUT_3_3        = 0x07
TCXO_DELAY_2_5         = 0x0140    # 2.5 ms
TCXO_DELAY_5           = 0x0280    # 5   ms
TCXO_DELAY_10          = 0x0560    # 10  ms

# ---- DIO2 mode ----
DIO2_AS_IRQ            = 0x00
DIO2_AS_RF_SWITCH      = 0x01

# ---- IRQ masks ----
IRQ_TX_DONE            = 0x0001
IRQ_RX_DONE            = 0x0002
IRQ_PREAMBLE_DETECTED  = 0x0004
IRQ_SYNC_WORD_VALID    = 0x0008
IRQ_HEADER_VALID       = 0x0010
IRQ_HEADER_ERR         = 0x0020
IRQ_CRC_ERR            = 0x0040
IRQ_CAD_DONE           = 0x0080
IRQ_CAD_DETECTED       = 0x0100
IRQ_TIMEOUT            = 0x0200
IRQ_ALL                = 0x03FF
IRQ_NONE               = 0x0000

# ---- LoRa bandwidth codes ----
BW_7800                = 0x00
BW_10400               = 0x08
BW_15600               = 0x01
BW_20800               = 0x09
BW_31250               = 0x02
BW_41700               = 0x0A
BW_62500               = 0x03
BW_125000              = 0x04
BW_250000              = 0x05
BW_500000              = 0x06

# ---- LoRa coding rate selector (offset from CR=4/4) ----
# cr parameter encodes (4 / (4+n)); LoRaRF subtracts 4 from user input
CR_OFFSET              = 4
LDRO_OFF               = 0x00
LDRO_ON                = 0x01

# ---- Packet params ----
HEADER_EXPLICIT        = 0x00
HEADER_IMPLICIT        = 0x01
CRC_OFF                = 0x00
CRC_ON                 = 0x01
IQ_STANDARD            = 0x00
IQ_INVERTED            = 0x01

# ---- PA ramp times ----
PA_RAMP_10U            = 0x00
PA_RAMP_20U            = 0x01
PA_RAMP_40U            = 0x02
PA_RAMP_80U            = 0x03
PA_RAMP_200U           = 0x04
PA_RAMP_800U           = 0x05
PA_RAMP_1700U          = 0x06
PA_RAMP_3400U          = 0x07

# ---- Regulator mode ----
REGULATOR_LDO          = 0x00
REGULATOR_DC_DC        = 0x01

# ---- CalibrateImage band presets ----
CAL_IMG_430            = 0x6B
CAL_IMG_440            = 0x6F
CAL_IMG_470            = 0x75
CAL_IMG_510            = 0x81
CAL_IMG_779            = 0xC1
CAL_IMG_787            = 0xC5
CAL_IMG_863            = 0xD7
CAL_IMG_870            = 0xDB
CAL_IMG_902            = 0xE1
CAL_IMG_928            = 0xE9

# ---- Rx/Tx fallback mode ----
FALLBACK_FS            = 0x40
FALLBACK_STDBY_XOSC    = 0x30
FALLBACK_STDBY_RC      = 0x20

# ---- GetStatus reply decoding ----
STATUS_DATA_AVAILABLE  = 0x04
STATUS_CMD_TIMEOUT     = 0x06
STATUS_CMD_ERROR       = 0x08
STATUS_CMD_FAILED      = 0x0A
STATUS_CMD_TX_DONE     = 0x0C
STATUS_MODE_STDBY_RC   = 0x20
STATUS_MODE_STDBY_XOSC = 0x30
STATUS_MODE_FS         = 0x40
STATUS_MODE_RX         = 0x50
STATUS_MODE_TX         = 0x60

# ---- High-level op-result codes (used by SX126xInterface layer) ----
OP_STATUS_DEFAULT      = 0
OP_STATUS_TX_WAIT      = 1
OP_STATUS_TX_TIMEOUT   = 2
OP_STATUS_TX_DONE      = 3
OP_STATUS_RX_WAIT      = 4
OP_STATUS_RX_CONTINUOUS= 5
OP_STATUS_RX_TIMEOUT   = 6
OP_STATUS_RX_DONE      = 7
OP_STATUS_HEADER_ERR    = 8
OP_STATUS_CRC_ERR      = 9
OP_STATUS_CAD_WAIT     = 10
OP_STATUS_CAD_DETECTED = 11
OP_STATUS_CAD_DONE     = 12

# ---- Key registers (copied verbatim from LoRaRF) ----
REG_TX_MODULATION      = 0x0889
REG_RX_GAIN            = 0x08AC
REG_TX_CLAMP_CONFIG    = 0x08D8
REG_OCP_CONFIGURATION  = 0x08E7
REG_RTC_CONTROL        = 0x0902
REG_EVENT_MASK         = 0x0944
REG_LORA_SYNC_WORD_MSB = 0x0740
REG_IQ_POLARITY_SETUP  = 0x0736

# ---- Rx gain presets ----
RX_GAIN_POWER_SAVING   = 0x00
RX_GAIN_BOOSTED        = 0x01
POWER_SAVING_GAIN      = 0x94
BOOSTED_GAIN           = 0x96

# ---- Frequency synthesis constants (from LoRaRF) ----
RF_FREQUENCY_XTAL      = 32000000
RF_FREQUENCY_NOM       = 33554432   # 2^25

# ---- SetRx / SetTx timeout encodings ----
RX_CONTINUOUS          = 0xFFFFFF   # "infinite" Rx timeout (continuous)
TX_SINGLE              = 0x000000   # no Tx timeout (single-shot)


# ---------------------------------------------------------------------------
# Bit width / band tables used by the high-level helpers. Kept module-level
# so the helpers stay small and the table is easy to audit.
# ---------------------------------------------------------------------------

# Maps (bandwidth in Hz) -> the SX126x SetModulationParams bandwidth code.
_BW_HZ_TO_CODE = [
    (9100,    BW_7800),
    (13000,   BW_10400),
    (18200,   BW_15600),
    (26000,   BW_20800),
    (36500,   BW_31250),
    (52100,   BW_41700),
    (93800,   BW_62500),
    (187500,  BW_125000),
    (375000,  BW_250000),
    (10**12,  BW_500000),  # sentinel: anything >= 375kHz is 500kHz
]

# Maps band range to image-calibration pair (matches LoRaRF.setFrequency)
_BAND_CALIBRATION = [
    (446000000, CAL_IMG_430, CAL_IMG_440),
    (734000000, CAL_IMG_470, CAL_IMG_510),
    (828000000, CAL_IMG_779, CAL_IMG_787),
    (877000000, CAL_IMG_863, CAL_IMG_870),
    (10**18,    CAL_IMG_902, CAL_IMG_928),
]


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class SX126xRadio:
    """Minimal SX1261/SX1262/SX1268 / LLCC68 driver on spidev + libgpiod 2.2.x.

    All public methods map 1:1 to an SX126x command (or to a small composition
    of commands, clearly named). No threading, no queues, no callbacks — the
    caller drives the chip and is responsible for the high-level TX/RX state
    machine. This class owns:

      * /dev/spidevX.Y                — opened with mode 0 (CPOL=0, CPHA=0)
      * libgpiod v2 LineRequest handles — one independent request per pin
        (RESET, BUSY, IRQ (edge), TXEN, RXEN, CS) — for RESET, BUSY, IRQ
        (edge), TXEN, RXEN
      * the SX126x chip state         — kept consistent with SX126x datasheet
        rev 1.2

    The IRQ wait (wait_irq_done) blocks on a libgpiod edge event, *not* a CPU
    spin, which is the whole reason this driver exists.

    libgpiod v2 note: unlike v1's Chip.get_line(offset) -> persistent Line
    object, v2 is request-based: gpiod.request_lines(chip_path, config={...})
    returns a LineRequest that can cover one or more offsets. Every pin here
    gets its OWN single-line request so each can be independently released/
    reconfigured (the IRQ pin in particular needs reconfigure_lines() to add
    edge detection after being requested as a plain input at open() time).
    request.set_value()/get_value() therefore take the pin OFFSET as their
    first argument (a request can cover multiple lines), unlike v1's
    line.set_value()/get_value() which took none.
    """

    # ---- pin value polarity: gpiod.line.Value enum members, not raw ints ---
    LOW  = _GpiodValue.INACTIVE if _GpiodValue is not None else 0
    HIGH = _GpiodValue.ACTIVE if _GpiodValue is not None else 1

    def __init__(self,
                 spi_bus=0,
                 spi_cs=0,
                 spi_speed=2_000_000,
                 pin_reset=18,
                 pin_busy=20,
                 pin_irq=16,
                 pin_txen=-1,
                 pin_rxen=-1,
                 pin_cs=-1,
                 gpiochip="gpiochip0",
                 pin_gpiochips=None,
                 txen_active_low=False,
                 rxen_active_low=False,
                 dio3_tcxo_voltage=None,
                 dio3_tcxo_delay_ms=5,
                 busy_timeout_ms=5000,
                 ):
        """Configure but do *not* open any hardware yet. Call open() to bring
        the chip up; call close() to put it to sleep and release resources.

        pin_cs : gpiochip line offset for the SX126x NSS (active-low chip
                 select) line. Set to >= 0 to enable bit-banged CS over
                 libgpiod. This is REQUIRED on platforms where the SPI
                 controller has no hardware CS pin wired (e.g. Raspberry Pi
                 with the `spi0-0cs` device-tree overlay, where spidev's
                 hardware-CE toggling goes nowhere at the silicon level).
                 Set to -1 to defer CS entirely to spidev's hardware-CE
                 path (works when the SPI controller's CE pin is wired to
                 the chip's NSS, e.g. some onboard SX126x modules with a
                 hardwired CE<->NSS connection, or via
                 `dtoverlay=spi0-1cs,cs0_pin=<n>`).

        pin_gpiochips : optional dict {"cs"|"reset"|"busy"|"irq"|"txen"|"rxen":
                 gpiochip_path}. Most boards wire every control pin to the
                 SAME gpiochip, in which case the single `gpiochip` argument
                 is all that's needed and this can be left as None. Some
                 boards (e.g. the Luckfox Lyra Zero W) wire different pins
                 to DIFFERENT gpiochips (RESET/BUSY on gpiochip0, IRQ/TXEN/
                 RXEN on gpiochip1) - pass a per-pin override here for those.
                 Any pin field missing from this dict falls back to the
                 single `gpiochip` default. Discovered live: without this,
                 a pin whose line OFFSET happens to coincide with an
                 already-claimed line on the WRONG chip fails with a
                 confusing "Device or resource busy" that looks like a real
                 hardware conflict but is actually just asking the wrong
                 gpiochip for that offset."""
        if spidev is None:
            raise SX126xError("spidev module not available; install python3-spidev")
        if gpiod is None:
            raise SX126xError("gpiod module not available; install python3-libgpiod")

        self.spi_bus       = spi_bus
        self.spi_cs        = spi_cs
        self.spi_speed     = spi_speed
        self.pin_reset     = pin_reset
        self.pin_busy      = pin_busy
        self.pin_irq       = pin_irq
        self.pin_txen      = pin_txen
        self.pin_rxen      = pin_rxen
        self.pin_cs        = pin_cs
        # Polarity of the txen/rxen "enabled" state. Most boards seen so far
        # (e.g. MeshAdv-style TXEN/RXEN pairs) are active-HIGH for both, but
        # the Station G3's LNA enable pin is active-LOW (LNA ON = logic 0,
        # per BQ's own lna_control.sh script) - so this can't be assumed.
        # Kept as separate flags (not folded into pin_gpiochips) since
        # polarity is a property of the SIGNAL, not of which gpiochip it
        # lives on.
        self.txen_active_low = txen_active_low
        self.rxen_active_low = rxen_active_low

        # libgpiod v2's gpiod.request_lines()/gpiod.Chip() require a full
        # device path (e.g. "/dev/gpiochip0") — a bare chip name like
        # "gpiochip0" raises FileNotFoundError (v1's Chip.get_line() API
        # accepted bare names, so board/platform profiles historically
        # specified them without the "/dev/" prefix). Normalize here, once,
        # at the point of construction, rather than requiring every profile
        # table entry to be rewritten with the full path.
        def _normalize_chip_path(name):
            if isinstance(name, str) and not name.startswith("/dev/"):
                return "/dev/" + name
            return name

        self.gpiochip_name = _normalize_chip_path(gpiochip)
        _chips = pin_gpiochips or {}
        self.gpiochip_cs    = _normalize_chip_path(_chips.get("cs", gpiochip))
        self.gpiochip_reset = _normalize_chip_path(_chips.get("reset", gpiochip))
        self.gpiochip_busy  = _normalize_chip_path(_chips.get("busy", gpiochip))
        self.gpiochip_irq   = _normalize_chip_path(_chips.get("irq", gpiochip))
        self.gpiochip_txen  = _normalize_chip_path(_chips.get("txen", gpiochip))
        self.gpiochip_rxen  = _normalize_chip_path(_chips.get("rxen", gpiochip))
        self.dio3_tcxo_voltage = dio3_tcxo_voltage
        self.dio3_tcxo_delay_ms = dio3_tcxo_delay_ms
        self.busy_timeout_ms = busy_timeout_ms

        # Hardware handles (None until open())
        self._spi          = None
        self._line_reset   = None
        self._line_busy    = None
        self._line_irq     = None
        self._line_txen    = None
        self._line_rxen    = None
        self._line_cs      = None   # bit-banged NSS if pin_cs >= 0

        # IRQ wait synchronization: a single consumer is expected per
        # wait_irq_done() call (matching LoRaRF's single-shot TX/RX model).
        self._irq_lock     = threading.Lock()
        self._irq_requested = False   # True once request_irq_edge() has run

        # Saved TX/RX-enable pin states so callers can restore them later.
        # (Matches LoRaRF's _txState / _rxState behaviour.)
        self._tx_state = self.LOW
        self._rx_state = self.LOW

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @staticmethod
    def _request_lines_retrying(chip_path, consumer, config, retries=5, delay_s=0.05):
        """gpiod.request_lines() wrapper that retries on OSError (errno 16,
        "Device or resource busy"). Observed live on the RK3506B vendor
        kernel: individual GPIO lines can transiently report busy for a few
        tens of milliseconds right after another pin on the SoC is toggled
        (e.g. immediately after the RESET pin bounce), even though nothing
        else ever shows up as holding the line (checked via
        gpiod.Chip.get_line_info() and /sys/kernel/debug/pinctrl - both
        report the pin as fully unclaimed within a second of the failure).
        This is *not* a fix for a real external conflict - if the line is
        genuinely held by another process/driver forever, this will still
        raise after exhausting retries."""
        last_exc = None
        for attempt in range(retries):
            try:
                return gpiod.request_lines(chip_path, consumer=consumer, config=config)
            except OSError as exc:
                if getattr(exc, "errno", None) != 16:  # not EBUSY - don't retry
                    raise
                last_exc = exc
                # DIAGNOSTIC (temporary): dump the chip's own view of every
                # line's holder from INSIDE this same process, before
                # anything unwinds/releases. If this process itself already
                # holds the offset (e.g. a duplicate open() call), the
                # consumer string here will be one of ours ("sx126x-*").
                try:
                    with gpiod.Chip(chip_path) as _diag_chip:
                        for off in range(32):
                            info = _diag_chip.get_line_info(off)
                            if info.used:
                                print(f"[gpio-diag] {chip_path} line {off}: "
                                      f"used=True consumer={info.consumer!r}",
                                      flush=True)
                except Exception as diag_exc:
                    print(f"[gpio-diag] failed: {diag_exc!r}", flush=True)
                if attempt < retries - 1:
                    time.sleep(delay_s)
        raise last_exc

    def open(self):
        """Open SPI, claim GPIO lines, hardware-reset the chip, and put it
        into STDBY_RC with the LoRa modem selected. Returns True on success."""

        # --- SPI ---
        self._spi = spidev.SpiDev()
        self._spi.open(self.spi_bus, self.spi_cs)
        self._spi.max_speed_hz = self.spi_speed
        self._spi.mode = 0          # CPOL=0, CPHA=0 per SX126x datasheet
        self._spi.lsbfirst = False   # MSB first
        # If we're bit-banging CS via libgpiod, tell spidev NOT to drive its
        # own (unwired) hardware CE pin. spidev toggles the CE pin anyway
        # by default; setting no_cs=True suppresses that to keep the bus
        # state clean and predictable. Safe even when spidev's CE pin is
        # wired to NSS — the bit-banged line wins regardless.
        if self.pin_cs is not None and self.pin_cs >= 0:
            try:
                self._spi.no_cs = True
            except Exception:
                # Older spidev versions may not support this attribute; in
                # that case spidev's hardware CE toggles a pin that goes
                # nowhere on a `spi0-0cs` overlay, so it's harmless.
                pass

        # --- GPIO lines (libgpiod v2: request-based, no persistent Chip) ---
        # Each pin gets its OWN single-line request via the top-level
        # gpiod.request_lines(chip_path, config={offset: LineSettings(...)})
        # convenience function, rather than v1's Chip.get_line(offset) then
        # line.request(). gpiochip_name must be a full device path (e.g.
        # "/dev/gpiochip0") under v2 - a bare "gpiochip0" name raises
        # FileNotFoundError.

        # --- CS / NSS (active-low chip-select, driven LOW per SPI transaction
        #     if pin_cs was configured; otherwise spidev's hardware-CE handles
        #     it). Idle state is HIGH so the SX126x ignores the bus between
        #     transactions. ---
        if self.pin_cs is not None and self.pin_cs >= 0:
            self._line_cs = self._request_lines_retrying(
                self.gpiochip_cs,
                consumer="sx126x-cs",
                config={self.pin_cs: gpiod.LineSettings(
                    direction=_GpiodDirection.OUTPUT,
                    output_value=self.HIGH,   # idle (de-asserted)
                )},
            )

        # --- RESET (output, default high so we start de-asserted) ---
        self._line_reset = self._request_lines_retrying(
            self.gpiochip_reset,
            consumer="sx126x-reset",
            config={self.pin_reset: gpiod.LineSettings(
                direction=_GpiodDirection.OUTPUT,
                output_value=self.HIGH,
            )},
        )

        # --- BUSY (input, no pull — BUSY is push-pull from the chip) ---
        self._line_busy = self._request_lines_retrying(
            self.gpiochip_busy,
            consumer="sx126x-busy",
            config={self.pin_busy: gpiod.LineSettings(direction=_GpiodDirection.INPUT)},
        )

        # --- IRQ (input; not requested as edge until request_irq_edge()) ---
        if self.pin_irq is not None and self.pin_irq >= 0:
            # Don't request edge detection yet — caller decides when, via
            # reconfigure_lines() in request_irq_edge(). Request as a plain
            # input first so it's claimed/known.
            self._line_irq = self._request_lines_retrying(
                self.gpiochip_irq,
                consumer="sx126x-irq",
                config={self.pin_irq: gpiod.LineSettings(direction=_GpiodDirection.INPUT)},
            )

        # --- TXEN / RXEN (optional outputs) ---
        if self.pin_txen is not None and self.pin_txen >= 0:
            self._line_txen = self._request_lines_retrying(
                self.gpiochip_txen,
                consumer="sx126x-txen",
                config={self.pin_txen: gpiod.LineSettings(
                    direction=_GpiodDirection.OUTPUT,
                    output_value=self.LOW,
                )},
            )
        if self.pin_rxen is not None and self.pin_rxen >= 0:
            self._line_rxen = self._request_lines_retrying(
                self.gpiochip_rxen,
                consumer="sx126x-rxen",
                config={self.pin_rxen: gpiod.LineSettings(
                    direction=_GpiodDirection.OUTPUT,
                    output_value=self.LOW,
                )},
            )

        # --- Hardware reset sequence (per SX126x datasheet §4.1) ---
        # Drive RESET low, hold >= 100us, drive high, then wait for BUSY low.
        self._line_reset.set_value(self.pin_reset, self.LOW)
        time.sleep(0.001)              # 1 ms; LoRaRF uses 1 ms and it works
        self._line_reset.set_value(self.pin_reset, self.HIGH)
        self._wait_busy(self.busy_timeout_ms)

        # --- Sanity check: chip responds to SetStandby + GetStatus ---
        self.set_standby(STANDBY_RC)
        if self.get_status_and_mode() != STATUS_MODE_STDBY_RC:
            self.close()
            raise SX126xError(
                "SX126x not responding after reset (status=0x{:02x})".format(
                    self.get_status_and_mode()))

        self.set_packet_type(PACKET_TYPE_LORA)
        self._fix_resistance_antenna()

        # --- Optional DIO3-as-TCXO bring-up ---
        if self.dio3_tcxo_voltage is not None:
            voltage_code = self._tcxo_voltage_code(self.dio3_tcxo_voltage)
            delay_code = self._tcxo_delay_code(self.dio3_tcxo_delay_ms)
            self.set_dio3_as_tcxo_ctrl(voltage_code, delay_code)
            # Per datasheet: after enabling TCXO, you must SetStandby and
            # Calibrate again so the chip knows the XOSC is now warm.
            self.set_standby(STANDBY_RC)
            self.calibrate(0xFF)
            # XOSC_START_ERR latches in the device-errors register on every
            # cold start on TCXO boards (datasheet §13.5.13). Clear it so
            # future diagnostics aren't confused by a stale latched error
            # from this init.
            try:
                self.clear_device_errors()
            except Exception:
                pass

        return True

    def close(self):
        """Sleep the chip and release every hardware handle. Safe to call
        multiple times."""
        # Try to put the chip to sleep first; ignore errors so we always
        # release handles.
        if self._spi is not None:
            try:
                self.set_standby(STANDBY_RC)
                self.set_sleep(SLEEP_COLD_START)
            except Exception:
                pass

        for line in (self._line_cs,    # release before the others so the
                     self._line_irq, self._line_txen, self._line_rxen,
                     self._line_busy, self._line_reset):
            if line is not None:
                try:
                    line.release()  # LineRequest.release() - same name in v2
                except Exception:
                    pass

        if self._spi is not None:
            try:
                self._spi.close()
            except Exception:
                pass

        self._line_reset = self._line_busy = self._line_irq = None
        self._line_txen  = self._line_rxen  = None
        self._line_cs    = None
        self._spi        = None

    # ------------------------------------------------------------------
    # Hardware reset (public; can be called again at runtime)
    # ------------------------------------------------------------------

    def reset(self):
        """Toggle the RESET pin per SX126x datasheet §4.1 and wait for BUSY
        to go low (with busy_timeout_ms budget)."""
        if self._line_reset is None:
            raise SX126xError("reset() called before open()")
        self._line_reset.set_value(self.pin_reset, self.LOW)
        time.sleep(0.001)
        self._line_reset.set_value(self.pin_reset, self.HIGH)
        self._wait_busy(self.busy_timeout_ms)

    # ------------------------------------------------------------------
    # SPI plumbing
    # ------------------------------------------------------------------

    def _wait_busy(self, timeout_ms):
        """Block (with a short sleep) until BUSY goes low or we time out.
        Returns True if BUSY went low in time, False on timeout."""
        deadline = time.monotonic() + (timeout_ms / 1000.0)
        # Most BUSY transitions finish in <<1ms, but the chip can hold BUSY
        # for several ms after SetDio3AsTcxoCtrl while the TCXO warms.
        # A 1ms sleep keeps CPU usage negligible vs. a true busy-spin.
        while self._line_busy.get_value(self.pin_busy) == self.HIGH:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.001)
        return True

    def _spi_write(self, opcode, data=b""):
        """Issue a write-only SX126x command: [opcode, data...].
        Blocks on BUSY first (mandatory before any SPI transaction)."""
        if self._wait_busy(self.busy_timeout_ms) is False:
            raise SX126xTimeout("BUSY pin stayed high before opcode 0x{:02x}".format(opcode))
        buf = bytes([opcode]) + (data if isinstance(data, (bytes, bytearray)) else bytes(data))
        if self._line_cs is not None:
            # Bit-banged CS (active-low). Assert LOW, transaction, deassert HIGH.
            self._line_cs.set_value(self.pin_cs, self.LOW)
            try:
                self._spi.xfer2(list(buf))
            finally:
                self._line_cs.set_value(self.pin_cs, self.HIGH)
        else:
            self._spi.xfer2(list(buf))

    def _spi_read(self, opcode, address=b"", n_data=1):
        """Issue a read SX126x command: [opcode, address..., 0x00 * (n_data+1)].
        The SX126x SPI read protocol requires ONE extra dummy byte beyond the
        address and data — the chip clocks out one STATUS byte per MISO byte
        until the data phase starts. The wire layout is:

            MOSI:  [opcode, addr0..addrN, 0x00, 0x00..(n_data-1)]
            MISO:  [STATUS, addr0_echo..addrN_echo, 0x00, DATA[0..n_data-1]]

        We send (n_data + 1) trailing zero bytes so that the chip clocks out
        (n_data + 1) MISO bytes in the data phase, and slice the last
        n_data bytes from the feedback. Slice offset is therefore
        n_addr + 2 (skip opcode status, skip address echoes, then data).

        The previous version sent only n_data trailing zeros and sliced from
        n_addr + 1, which made every read return a value shifted by one byte
        and corrupted read-modify-write sequences (REG_TX_MODULATION,
        REG_TX_CLAMP_CONFIG, REG_IQ_POLARITY_SETUP) — the chip was
        repeatedly written back with the wrong byte, eventually leaving
        the TX path in a state the chip would refuse to execute (status 0x2a,
        "command execution failed")."""
        if self._wait_busy(self.busy_timeout_ms) is False:
            raise SX126xTimeout("BUSY pin stayed high before opcode 0x{:02x}".format(opcode))
        n_addr = len(address)
        buf = bytes([opcode]) + bytes(address) + (b"\x00" * (n_data + 1))
        if self._line_cs is not None:
            self._line_cs.set_value(self.pin_cs, self.LOW)
            try:
                feedback = self._spi.xfer2(list(buf))
            finally:
                self._line_cs.set_value(self.pin_cs, self.HIGH)
        else:
            feedback = self._spi.xfer2(list(buf))
        # Layout of feedback:
        #   feedback[0]            = STATUS byte (auto-clocked on every read)
        #   feedback[1..n_addr]    = echoed address bytes
        #   feedback[n_addr+1]     = 0x00 (the extra "status NOP" we clocked out)
        #   feedback[n_addr+2:]    = real DATA bytes
        return bytes(feedback[n_addr + 2:])

    # ------------------------------------------------------------------
    # Operational mode commands (§13.1 of SX126x datasheet)
    # ------------------------------------------------------------------

    def set_sleep(self, sleep_config=SLEEP_COLD_START):
        self._spi_write(CMD_SET_SLEEP, bytes([sleep_config]))

    def set_standby(self, standby_config=STANDBY_RC):
        self._spi_write(CMD_SET_STANDBY, bytes([standby_config]))

    def set_fs(self):
        self._spi_write(CMD_SET_FS)

    def set_tx(self, timeout=0x000000):
        """Start transmitting. `timeout` is encoded in 15.625us units;
        LoRaRF pre-shifts it (<< 6); we accept the already-shifted value to
        match the LoRaRF behaviour the rest of the codebase already uses."""
        self._spi_write(CMD_SET_TX, bytes([
            (timeout >> 16) & 0xFF,
            (timeout >> 8)  & 0xFF,
            timeout         & 0xFF,
        ]))

    def set_rx(self, timeout=0xFFFFFF):
        """Start receiving. `timeout` semantics match set_tx()."""
        self._spi_write(CMD_SET_RX, bytes([
            (timeout >> 16) & 0xFF,
            (timeout >> 8)  & 0xFF,
            timeout         & 0xFF,
        ]))

    def set_timer_on_preamble(self, enable):
        self._spi_write(CMD_SET_TIMER_ON_PREAMBLE, bytes([enable & 0xFF]))

    def set_rx_duty_cycle(self, rx_period, sleep_period):
        self._spi_write(CMD_SET_RX_DUTY_CYCLE, bytes([
            (rx_period    >> 16) & 0xFF,
            (rx_period    >> 8)  & 0xFF,
            rx_period            & 0xFF,
            (sleep_period >> 16) & 0xFF,
            (sleep_period >> 8)  & 0xFF,
            sleep_period         & 0xFF,
        ]))

    def set_cad(self):
        self._spi_write(CMD_SET_CAD)

    def set_tx_continuous_wave(self):
        self._spi_write(CMD_SET_TX_CONTINUOUS_WAVE)

    def set_tx_infinite_preamble(self):
        self._spi_write(CMD_SET_TX_INFINITE_PREAMBLE)

    def set_regulator_mode(self, mode):
        self._spi_write(CMD_SET_REGULATOR_MODE, bytes([mode & 0xFF]))

    def calibrate(self, calib_param):
        self._spi_write(CMD_CALIBRATE, bytes([calib_param & 0xFF]))

    def calibrate_image(self, freq1, freq2):
        self._spi_write(CMD_CALIBRATE_IMAGE, bytes([freq1 & 0xFF, freq2 & 0xFF]))

    def set_pa_config(self, pa_duty_cycle, hp_max, device_sel, pa_lut):
        self._spi_write(CMD_SET_PA_CONFIG, bytes([
            pa_duty_cycle & 0xFF,
            hp_max        & 0xFF,
            device_sel    & 0xFF,
            pa_lut        & 0xFF,
        ]))

    def set_rx_tx_fallback_mode(self, fallback_mode):
        self._spi_write(CMD_SET_RX_TX_FALLBACK_MODE, bytes([fallback_mode & 0xFF]))

    # ------------------------------------------------------------------
    # Register / buffer access (§13.2)
    # ------------------------------------------------------------------

    def write_register(self, address, data):
        """Write 1..N bytes to a 16-bit register address (MSB first)."""
        if not 0 <= address <= 0xFFFF:
            raise ValueError("register address out of range")
        buf = bytes([(address >> 8) & 0xFF, address & 0xFF]) + bytes(data)
        self._spi_write(CMD_WRITE_REGISTER, buf)

    def read_register(self, address, n_bytes=1):
        """Read 1..N bytes from a 16-bit register address (MSB first)."""
        if not 0 <= address <= 0xFFFF:
            raise ValueError("register address out of range")
        addr = bytes([(address >> 8) & 0xFF, address & 0xFF])
        return self._spi_read(CMD_READ_REGISTER, address=addr, n_data=n_bytes)

    def write_buffer(self, offset, data):
        """Write `data` to the radio's 256-byte buffer starting at `offset`."""
        buf = bytes([offset & 0xFF]) + bytes(data)
        self._spi_write(CMD_WRITE_BUFFER, buf)

    def read_buffer(self, offset, n_bytes):
        """Read `n_bytes` from the radio's 256-byte buffer starting at `offset`."""
        return self._spi_read(CMD_READ_BUFFER, address=bytes([offset & 0xFF]), n_data=n_bytes)

    def set_buffer_base_address(self, tx_base, rx_base):
        self._spi_write(CMD_SET_BUFFER_BASE_ADDRESS, bytes([tx_base & 0xFF, rx_base & 0xFF]))

    # ------------------------------------------------------------------
    # DIO / IRQ control (§13.3)
    # ------------------------------------------------------------------

    def set_dio_irq_params(self, irq_mask, dio1_mask=0, dio2_mask=0, dio3_mask=0):
        """Configure which IRQ sources map to which DIO pins. All four masks
        are 10-bit values packed little-endian into 8 bytes (LoRaRF layout)."""
        buf = bytes([
            (irq_mask  >> 8) & 0xFF, irq_mask  & 0xFF,
            (dio1_mask >> 8) & 0xFF, dio1_mask & 0xFF,
            (dio2_mask >> 8) & 0xFF, dio2_mask & 0xFF,
            (dio3_mask >> 8) & 0xFF, dio3_mask & 0xFF,
        ])
        self._spi_write(CMD_SET_DIO_IRQ_PARAMS, buf)

    def get_irq_status(self):
        """Return the 16-bit IRQ status word (only the lower 10 bits are used)."""
        raw = self._spi_read(CMD_GET_IRQ_STATUS, n_data=2)
        return ((raw[0] << 8) | raw[1]) & 0x03FF

    def clear_irq_status(self, irq_mask):
        """Acknowledge / clear the given IRQ sources (write-1-to-clear)."""
        self._spi_write(CMD_CLEAR_IRQ_STATUS, bytes([
            (irq_mask >> 8) & 0xFF,
            irq_mask        & 0xFF,
        ]))

    def set_dio2_as_rf_switch_ctrl(self, enable=True):
        """If enable=True, route DIO2 as RF switch control (for E22-style
        modules that use DIO2 instead of dedicated TXEN/RXEN GPIOs)."""
        self._spi_write(CMD_SET_DIO2_AS_RF_SWITCH_CTRL,
                        bytes([DIO2_AS_RF_SWITCH if enable else DIO2_AS_IRQ]))

    def set_dio3_as_tcxo_ctrl(self, voltage, delay):
        """Configure DIO3 to supply a TCXO reference. voltage is one of the
        DIO3_OUTPUT_* codes; delay is the SX126x delay code (TCXO_DELAY_*
        or a computed value in 15.625us units)."""
        self._spi_write(CMD_SET_DIO3_AS_TCXO_CTRL, bytes([
            voltage            & 0xFF,
            (delay >> 16) & 0xFF,
            (delay >> 8)  & 0xFF,
            delay         & 0xFF,
        ]))

    # ------------------------------------------------------------------
    # RF / modulation / packet parameters (§13.4)
    # ------------------------------------------------------------------

    def set_rf_frequency(self, rf_freq):
        """rf_freq is the raw 32-bit value computed as
        freq_hz * 2^25 / 32_000_000."""
        self._spi_write(CMD_SET_RF_FREQUENCY, bytes([
            (rf_freq >> 24) & 0xFF,
            (rf_freq >> 16) & 0xFF,
            (rf_freq >> 8)  & 0xFF,
            rf_freq         & 0xFF,
        ]))

    def set_packet_type(self, packet_type):
        self._spi_write(CMD_SET_PACKET_TYPE, bytes([packet_type & 0xFF]))

    def get_packet_type(self):
        return self._spi_read(CMD_GET_PACKET_TYPE, n_data=1)[0]

    def set_tx_params(self, power, ramp_time=PA_RAMP_800U):
        """`power` is the 8-bit SX126x power register value (NOT dBm).
        Use set_tx_power() to convert from dBm."""
        self._spi_write(CMD_SET_TX_PARAMS, bytes([power & 0xFF, ramp_time & 0xFF]))

    def set_modulation_params_lora(self, sf, bw, cr, ldro):
        """Set LoRa modulation params. sf is 5..12, bw is one of the BW_*
        codes, cr is 1..4 (4/5 .. 4/8; we subtract CR_OFFSET internally so
        callers can pass cr=5..8 like the rest of the codebase does), ldro
        is LDRO_ON/LDRO_OFF."""
        self._spi_write(CMD_SET_MODULATION_PARAMS, bytes([
            sf    & 0xFF,
            bw    & 0xFF,
            cr    & 0xFF,
            ldro  & 0xFF,
            0, 0, 0, 0,        # reserved
        ]))

    def set_packet_params_lora(self, preamble_length, header_type, payload_length,
                                crc_type, invert_iq):
        self._spi_write(CMD_SET_PACKET_PARAMS, bytes([
            (preamble_length >> 8) & 0xFF,
            preamble_length        & 0xFF,
            header_type            & 0xFF,
            payload_length         & 0xFF,
            crc_type               & 0xFF,
            invert_iq              & 0xFF,
            0, 0, 0,               # reserved
        ]))

    def set_cad_params(self, cad_symbol_num, cad_det_peak, cad_det_min,
                       cad_exit_mode, cad_timeout):
        self._spi_write(CMD_SET_CAD_PARAMS, bytes([
            cad_symbol_num & 0xFF,
            cad_det_peak   & 0xFF,
            cad_det_min    & 0xFF,
            cad_exit_mode  & 0xFF,
            (cad_timeout >> 16) & 0xFF,
            (cad_timeout >> 8)  & 0xFF,
            cad_timeout         & 0xFF,
        ]))

    def set_lora_symb_num_timeout(self, symbnum):
        self._spi_write(CMD_SET_LORA_SYMB_NUM_TIMEOUT, bytes([symbnum & 0xFF]))

    # ------------------------------------------------------------------
    # Status commands (§13.5)
    # ------------------------------------------------------------------

    def get_status_byte(self):
        """Return the raw status byte from CMD_GET_STATUS."""
        return self._spi_read(CMD_GET_STATUS, n_data=1)[0]

    def get_status_and_mode(self):
        """Return just the chip-mode nibble (mask 0x70) of the status byte."""
        return self.get_status_byte() & 0x70

    def get_rx_buffer_status(self):
        """Return (payload_length, buffer_offset) for the last received packet."""
        raw = self._spi_read(CMD_GET_RX_BUFFER_STATUS, n_data=2)
        return raw[0], raw[1]

    def get_packet_status(self):
        """Return (rssi_pkt, snr_pkt, signal_rssi_pkt) raw register bytes.

        Conversion (per RNode / LoRaRF):
            rssi_dbm        = rssi_pkt / -2.0
            snr_db          = (snr_pkt if snr_pkt < 128 else snr_pkt - 256) / 4.0
            signal_rssi_dbm = signal_rssi_pkt / -2.0
        """
        raw = self._spi_read(CMD_GET_PACKET_STATUS, n_data=3)
        return raw[0], raw[1], raw[2]

    def get_rssi_inst(self):
        return self._spi_read(CMD_GET_RSSI_INST, n_data=1)[0]

    def get_device_errors(self):
        """Return the 16-bit device-errors register (OpErrors).
        Bit assignments per SX126x datasheet §13.5.13."""
        raw = self._spi_read(CMD_GET_DEVICE_ERRORS, n_data=2)
        return (raw[0] << 8) | raw[1]

    def clear_device_errors(self):
        self._spi_write(CMD_CLEAR_DEVICE_ERRORS, bytes([0, 0]))

    # ------------------------------------------------------------------
    # IRQ edge-wait (the whole reason this file exists)
    # ------------------------------------------------------------------

    def request_irq_edge(self, edge="rising"):
        """Reconfigure the IRQ line in-place to add edge detection so that
        wait_irq_done() can block on it instead of polling.

        edge may be "rising", "falling", or "both". The SX126x datasheet
        says IRQ is active-high and goes high when a configured event
        fires, so "rising" is the normal choice.

        libgpiod v2 note: unlike v1 (which required release() + a fresh
        request() to change the line's event mode), v2's
        LineRequest.reconfigure_lines() changes the existing request's
        settings in place — no release/re-request, no risk of a gap where
        the line briefly has no owner."""
        if self._line_irq is None:
            raise SX126xError("No IRQ pin configured (pin_irq is None or < 0)")
        if edge == "rising":
            edge_type = _GpiodEdge.RISING
        elif edge == "falling":
            edge_type = _GpiodEdge.FALLING
        elif edge == "both":
            edge_type = _GpiodEdge.BOTH
        else:
            raise ValueError("edge must be rising|falling|both")
        self._line_irq.reconfigure_lines(
            config={self.pin_irq: gpiod.LineSettings(
                direction=_GpiodDirection.INPUT,
                edge_detection=edge_type,
            )},
        )
        self._irq_requested = True

    def release_irq_edge(self):
        """Restore the IRQ line to a plain input (no event subscription).
        Safe to call even if request_irq_edge() was never called."""
        if self._line_irq is None:
            return
        try:
            self._line_irq.reconfigure_lines(
                config={self.pin_irq: gpiod.LineSettings(direction=_GpiodDirection.INPUT)},
            )
        except Exception:
            pass
        self._irq_requested = False

    def wait_irq_done(self, timeout_s):
        """Block until the IRQ line fires (rising edge) or `timeout_s`
        elapses.

        Returns the 16-bit IRQ status word (via GetIrqStatus) on success,
        or None on timeout. This is what the SX126xInterface layer will
        poll instead of LoRaRF's busy-spinning wait().

        If the IRQ pin is not wired (pin_irq is None/negative) or has not
        been request_irq_edge()'d, we fall back to a slow GetIrqStatus
        poll with a 5ms sleep — much cheaper than LoRaRF's busy spin but
        still not as good as the edge wait. This is intentional: it keeps
        the driver functional on boards where the IRQ pin isn't brought
        out to a usable GPIO."""
        deadline = time.monotonic() + max(timeout_s, 0.0)
        with self._irq_lock:
            if self._line_irq is not None and self._irq_requested:
                # Drain any backlog events that accumulated on the IRQ line
                # since the last call (e.g. from earlier SetStandby calls
                # that re-triggered DIO1, or from a previous wait_irq_done
                # that returned but didn't fully drain). Without this, a
                # stale FIFO entry can fire wait_edge_events() immediately
                # with an IRQ status from a previous operation.
                # (libgpiod v2.x: wait_edge_events(timeout) + read_edge_events();
                # timeout=0 is the non-blocking "is anything pending?" check.)
                while self._line_irq.wait_edge_events(0):
                    try:
                        self._line_irq.read_edge_events()
                    except Exception:
                        break
                # Edge-wait path — blocks in the kernel until the line fires.
                remaining = deadline - time.monotonic()
                if remaining < 0:
                    remaining = 0.0
                fired = self._line_irq.wait_edge_events(remaining)
                if not fired:
                    return None
                # Drain the event so the next wait_irq_done() can re-arm.
                try:
                    self._line_irq.read_edge_events()
                except Exception:
                    pass
                return self.get_irq_status()
            else:
                # Polling fallback for boards without a wired IRQ pin.
                # 5ms sleep ≈ 200Hz poll, which is plenty for LoRa packet
                # turnaround times (hundreds of ms typically) and burns
                # effectively zero CPU.
                while True:
                    irq = self.get_irq_status()
                    if irq != 0:
                        return irq
                    if time.monotonic() >= deadline:
                        return None
                    time.sleep(0.005)

    # ------------------------------------------------------------------
    # High-level composition helpers (used by SX126xInterface layer)
    # ------------------------------------------------------------------

    def set_frequency(self, frequency_hz):
        """Run the per-band image calibration and set the RF carrier
        frequency. Identical math to LoRaRF.setFrequency."""
        cal_freq1 = CAL_IMG_430
        cal_freq2 = CAL_IMG_440
        for upper, f1, f2 in _BAND_CALIBRATION:
            if frequency_hz < upper:
                cal_freq1 = f1
                cal_freq2 = f2
                break
        self.calibrate_image(cal_freq1, cal_freq2)
        rf_freq = int(frequency_hz * RF_FREQUENCY_NOM / RF_FREQUENCY_XTAL)
        self.set_rf_frequency(rf_freq)

    def set_tx_power(self, tx_power_dbm, version=TX_POWER_SX1262):
        """Convert dBm -> PA config bytes and SetTxParams. Matches the table
        in LoRaRF.setTxPower() byte-for-byte."""
        # Clamp to datasheet limits per chip variant.
        if tx_power_dbm > 22:
            tx_power_dbm = 22
        if version == TX_POWER_SX1261 and tx_power_dbm > 15:
            tx_power_dbm = 15

        pa_duty_cycle = 0x00
        hp_max        = 0x00
        device_sel    = 0x00 if version != TX_POWER_SX1261 else 0x01
        power         = 0x0E

        if version == TX_POWER_SX1261:
            device_sel = 0x01

        if tx_power_dbm == 22:
            pa_duty_cycle, hp_max, power = 0x04, 0x07, 0x16
        elif tx_power_dbm >= 20:
            pa_duty_cycle, hp_max, power = 0x03, 0x05, 0x16
        elif tx_power_dbm >= 17:
            pa_duty_cycle, hp_max, power = 0x02, 0x03, 0x16
        elif tx_power_dbm >= 14 and version == TX_POWER_SX1261:
            pa_duty_cycle, hp_max, power = 0x04, 0x00, 0x0E
        elif tx_power_dbm >= 14 and version == TX_POWER_SX1262:
            pa_duty_cycle, hp_max, power = 0x02, 0x02, 0x16
        elif tx_power_dbm >= 14 and version == TX_POWER_SX1268:
            pa_duty_cycle, hp_max, power = 0x04, 0x06, 0x0F
        elif tx_power_dbm >= 10 and version == TX_POWER_SX1261:
            pa_duty_cycle, hp_max, power = 0x01, 0x00, 0x0D
        elif tx_power_dbm >= 10 and version == TX_POWER_SX1268:
            pa_duty_cycle, hp_max, power = 0x00, 0x03, 0x0F
        else:
            # Below the lowest supported value (SX1262: 14 dBm,
            # SX1261/SX1268: 10 dBm). SX126x datasheet doesn't list a valid
            # SetTxParams + SetPaConfig pair for these values — calling it
            # with an inconsistent state (e.g. paDutyCycle=0 with a non-zero
            # power register) causes the chip to reject the next SetTx with
            # "command execution failed" (status 0x2a). Clamp UP to the
            # lowest supported value rather than silently doing nothing —
            # silently doing nothing would mean the chip retains its
            # post-reset default (22 dBm) and the user has no idea their
            # request was ignored.
            import warnings as _w
            _w.warn(
                "SX126x set_tx_power({} dBm) below minimum for this chip "
                "variant; clamping to 14 dBm.".format(tx_power_dbm),
                stacklevel=2,
            )
            # Re-run the resolution with the clamped value.
            if version == TX_POWER_SX1262:
                tx_power_dbm = 14
            else:
                tx_power_dbm = 10
            return self.set_tx_power(tx_power_dbm, version=version)

        self.set_pa_config(pa_duty_cycle, hp_max, device_sel, 0x01)
        self.set_tx_params(power, PA_RAMP_800U)

    def set_rx_gain(self, rx_gain):
        """rx_gain is RX_GAIN_POWER_SAVING (0x94 register value) or
        RX_GAIN_BOOSTED (0x96)."""
        gain = POWER_SAVING_GAIN if rx_gain == RX_GAIN_POWER_SAVING else BOOSTED_GAIN
        self.write_register(REG_RX_GAIN, bytes([gain]))
        if rx_gain == RX_GAIN_BOOSTED:
            # LoRaRF also writes a 3-byte retention register when boosted.
            self.write_register(0x029F, bytes([0x01, 0x08, 0xAC]))

    def set_lora_modulation(self, sf, bw_hz, cr, ldro=False):
        """High-level LoRa modulation setter. sf is 5..12, bw_hz is one
        of the Hz values (e.g. 125000), cr is 5..8, ldro is a bool."""
        # Clamp SF to datasheet limits.
        if sf > 12: sf = 12
        if sf < 5:  sf = 5

        # Map Hz -> SX126x bandwidth code.
        bw_code = BW_500000
        for upper, code in _BW_HZ_TO_CODE:
            if bw_hz < upper:
                bw_code = code
                break

        # CR is encoded as cr-4 in the chip register (4/5 -> 1, ... 4/8 -> 4)
        cr_code = cr - CR_OFFSET
        if cr_code < 0: cr_code = 0
        if cr_code > 4: cr_code = 0

        ldro_code = LDRO_ON if ldro else LDRO_OFF
        self.set_modulation_params_lora(sf, bw_code, cr_code, ldro_code)

        # Workaround for 500kHz LoRa bandwidth bit in REG_TX_MODULATION.
        # (Identical to LoRaRF._fixLoRaBw500.)
        packet_type = self.get_packet_type()
        cur = self.read_register(REG_TX_MODULATION, 1)[0]
        new = cur | 0x04
        if packet_type == PACKET_TYPE_LORA and bw_code == BW_500000:
            new = cur & 0xFB
        self.write_register(REG_TX_MODULATION, bytes([new]))

    def set_lora_packet(self, header_type, preamble_length, payload_length,
                         crc_type=True, invert_iq=False):
        """Set LoRa packet params (preamble, header mode, payload length,
        CRC on/off, IQ polarity) and apply the IQ-polarity workaround."""
        if header_type != HEADER_IMPLICIT:
            header_type = HEADER_EXPLICIT
        crc_code  = CRC_ON  if crc_type else CRC_OFF
        iq_code   = IQ_INVERTED if invert_iq else IQ_STANDARD

        self.set_packet_params_lora(preamble_length, header_type,
                                    payload_length, crc_code, iq_code)
        self._fix_inverted_iq(invert_iq)

    def set_sync_word(self, sync_word):
        """Set the LoRa sync word (REG_LORA_SYNC_WORD_MSB). Accepts a 16-bit
        integer (e.g. 0x1424 for the RNode-private word) — matching how
        LoRaRF.setSyncWord lays out the two register bytes."""
        if 0 <= sync_word <= 0xFF:
            # LoRaRF's compact form for 1-byte sync words
            buf = bytes([
                (sync_word & 0xF0) | 0x04,
                ((sync_word << 4) & 0xF0) | 0x04,
            ])
        else:
            buf = bytes([(sync_word >> 8) & 0xFF, sync_word & 0xFF])
        self.write_register(REG_LORA_SYNC_WORD_MSB, buf)

    # ------------------------------------------------------------------
    # TX/RX pin helpers (used by SX126xInterface to drive external
    # TXEN/RXEN GPIOs where the board doesn't rely on DIO2-as-RF-switch)
    # ------------------------------------------------------------------

    def _txen_level(self, enabled):
        """Resolve the raw HIGH/LOW to write to the TXEN pin for a logical
        enabled/disabled state, honoring txen_active_low."""
        if self.txen_active_low:
            return self.LOW if enabled else self.HIGH
        return self.HIGH if enabled else self.LOW

    def _rxen_level(self, enabled):
        """Same as _txen_level(), for the RXEN/LNA-enable pin."""
        if self.rxen_active_low:
            return self.LOW if enabled else self.HIGH
        return self.HIGH if enabled else self.LOW

    def set_tx_enable(self, on):
        """Drive the external TXEN pin (if wired). Saves the prior state
        so the caller can restore it after TX."""
        if self._line_txen is None:
            return
        if on:
            self._tx_state = self._line_txen.get_value(self.pin_txen)
            self._line_txen.set_value(self.pin_txen, self._txen_level(True))
            if self._line_rxen is not None:
                self._rx_state = self._line_rxen.get_value(self.pin_rxen)
                self._line_rxen.set_value(self.pin_rxen, self._rxen_level(False))

    def set_rx_enable(self, on):
        """Drive the external RXEN pin (if wired). Saves the prior state
        so the caller can restore it after RX."""
        if self._line_rxen is None:
            return
        if on:
            self._rx_state = self._line_rxen.get_value(self.pin_rxen)
            self._line_rxen.set_value(self.pin_rxen, self._rxen_level(True))
            if self._line_txen is not None:
                self._tx_state = self._line_txen.get_value(self.pin_txen)
                self._line_txen.set_value(self.pin_txen, self._txen_level(False))

    def restore_tx_rx_pins(self):
        """Restore TXEN/RXEN to whatever they were before the most recent
        set_tx_enable()/set_rx_enable() call. Matches LoRaRF's end-of-TX/RX
        behaviour."""
        if self._line_txen is not None:
            try:
                self._line_txen.set_value(self.pin_txen, self._tx_state)
            except Exception:
                pass
        if self._line_rxen is not None:
            try:
                self._line_rxen.set_value(self.pin_rxen, self._rx_state)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Internal workarounds (named verbatim from LoRaRF for traceability)
    # ------------------------------------------------------------------

    def _fix_resistance_antenna(self):
        """Set the OCP clamp register so the antenna-impedance workaround
        is enabled. Required after every (re)initialisation per LoRaRF."""
        cur = self.read_register(REG_TX_CLAMP_CONFIG, 1)[0]
        self.write_register(REG_TX_CLAMP_CONFIG, bytes([cur | 0x1E]))

    def _fix_inverted_iq(self, invert_iq):
        """Toggle the bit in REG_IQ_POLARITY_SETUP that the datasheet
        requires whenever IQ polarity is changed. Matches LoRaRF."""
        cur = self.read_register(REG_IQ_POLARITY_SETUP, 1)[0]
        new = (cur & 0xFB) | (0x04 if invert_iq else 0x00)
        self.write_register(REG_IQ_POLARITY_SETUP, bytes([new]))

    # ------------------------------------------------------------------
    # Static helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _tcxo_voltage_code(volts):
        mapping = {
            1.6: DIO3_OUTPUT_1_6,
            1.7: DIO3_OUTPUT_1_7,
            1.8: DIO3_OUTPUT_1_8,
            2.2: DIO3_OUTPUT_2_2,
            2.4: DIO3_OUTPUT_2_4,
            2.7: DIO3_OUTPUT_2_7,
            3.0: DIO3_OUTPUT_3_0,
            3.3: DIO3_OUTPUT_3_3,
        }
        # Pick the closest supported voltage.
        return mapping[min(mapping.keys(), key=lambda v: abs(v - volts))]

    @staticmethod
    def _tcxo_delay_code(delay_ms):
        """Convert a ms value into the SX126x TCXO delay register code
        (units of 15.625us; encoded as a 24-bit big-endian value)."""
        # The chip wants the value as (ms * 64) — see datasheet §13.3.6.
        return int(delay_ms * 64) & 0x00FFFFFF

    # ------------------------------------------------------------------
    # Convenience: blocking single-shot TX
    # ------------------------------------------------------------------

    def transmit_blocking(self, payload, irq_timeout_s=10.0, irq_mask=None):
        """High-level helper used by the SX126xInterface layer:
        1. Write `payload` into the TX buffer at offset 0
        2. Configure SetPacketParams with the right payload length
        3. Clear IRQ status, arm IRQ mask, call SetTx
        4. Block on wait_irq_done()
        5. Return the IRQ status word on completion / timeout
        Caller is responsible for IRQ-mask selection and for re-entering RX."""
        if irq_mask is None:
            irq_mask = IRQ_TX_DONE | IRQ_TIMEOUT
        self.clear_irq_status(IRQ_ALL)
        self.set_dio_irq_params(irq_mask, dio1_mask=irq_mask)
        self.write_buffer(0, payload)
        # NB: caller must have called set_packet_params_lora() with the
        # correct payload length BEFORE calling this. We don't touch
        # SetPacketParams here because the SX126xInterface layer has its
        # own state for header/CRC/IQ and it's already calling it once
        # per configuration.
        self.set_tx(0x000000)
        return self.wait_irq_done(irq_timeout_s)