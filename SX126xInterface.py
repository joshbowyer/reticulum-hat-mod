##############################################################################
# SX126xInterface.py                                                         #
#                                                                            #
# A Reticulum custom interface that drives SX1262/SX1268 LoRa chips         #
# directly over SPI on Linux SBCs (Raspberry Pi, Orange Pi, femtofox,       #
# etc). No separate RNode MCU required.                                      #
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
# Default pin config is for the MeshAdv Pi HAT v1.1:                        #
#   IRQ=16, Busy=20, Reset=18, TXen=13, RXen=12                             #
#                                                                            #
# Place SX126xInterface.py and vendored_sx126x.py in                         #
# ~/.reticulum/interfaces/ and add an interface config block to             #
# ~/.reticulum/config                                                        #
#                                                                            #
# Example config:                                                            #
#                                                                            #
#   [[MeshAdv LoRa]]                                                         #
#     type = SX126xInterface                                                 #
#     interface_enabled = True                                                #
#     frequency = 915000000                                                  #
#     bandwidth = 125000                                                     #
#     spreadingfactor = 8                                                    #
#     codingrate = 5                                                         #
#     txpower = 22                                                           #
#     spi_bus = 0                                                            #
#     spi_cs = 0                                                             #
#     pin_irq = 16                                                           #
#     pin_busy = 20                                                          #
#     pin_reset = 18                                                         #
#     pin_txen = 13                                                          #
#     pin_rxen = 12                                                          #
#     dio3_tcxo_voltage = 1.8                                                #
#                                                                            #
# License: MIT (matches the rest of this project)                            #
##############################################################################

# Custom interfaces loaded by Reticulum via exec() have "Interface" and "RNS"
# injected into their globals. We import additional stdlib modules we need
# here. Do NOT import vendored_sx126x at module top — see _load_vendored_driver
# for the explanation of why it's loaded inside __init__.
import os
import sys
import threading
import time
import math
import random
import queue
import importlib.util
from collections import deque


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

        # LoRa radio parameters
        frequency = int(c["frequency"]) if "frequency" in c else 915000000
        bandwidth = int(c["bandwidth"]) if "bandwidth" in c else 125000
        txpower   = int(c["txpower"])   if "txpower"   in c else 22
        sf        = int(c["spreadingfactor"]) if "spreadingfactor" in c else 8
        cr        = int(c["codingrate"]) if "codingrate" in c else 5

        # SPI bus configuration (spidev's chip-select index, NOT a GPIO)
        spi_bus = int(c["spi_bus"]) if "spi_bus" in c else 0
        spi_cs  = int(c["spi_cs"])  if "spi_cs"  in c else 0

        # GPIO pin configuration (BCM numbering)
        # MeshAdv Pi HAT v1.1 defaults.
        # NOTE: there is intentionally no `pin_cs` config key. The SX126x
        # chip-select is driven by the spidev driver using the SPI
        # controller's hardware CE line selected by `spi_cs` above; we do
        # not bit-bang CS via GPIO.
        pin_irq   = int(c["pin_irq"])   if "pin_irq"   in c else 16
        pin_busy  = int(c["pin_busy"])  if "pin_busy"  in c else 20
        pin_reset = int(c["pin_reset"]) if "pin_reset" in c else 18
        pin_txen  = int(c["pin_txen"])  if "pin_txen"  in c else 13
        pin_rxen  = int(c["pin_rxen"])  if "pin_rxen"  in c else 12

        # DIO3 TCXO voltage (set to 0 or None to disable)
        dio3_tcxo = float(c["dio3_tcxo_voltage"]) if "dio3_tcxo_voltage" in c else 1.8

        # Sync word: 0x12 = private/RNode, 0x34 = public/LoRaWAN.
        if "sync_word" in c:
            sw = c["sync_word"].strip()
            sync_word = int(sw, 16) if sw.startswith("0x") or sw.startswith("0X") else int(sw)
        else:
            sync_word = 0x12

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

        self.spi_bus   = spi_bus
        self.spi_cs    = spi_cs
        self.pin_irq   = pin_irq
        self.pin_busy  = pin_busy
        self.pin_reset = pin_reset
        self.pin_txen  = pin_txen
        self.pin_rxen  = pin_rxen
        self.dio3_tcxo = dio3_tcxo

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
            gpiochip="gpiochip0",
            dio3_tcxo_voltage=self.dio3_tcxo if self.dio3_tcxo and self.dio3_tcxo > 0 else None,
            dio3_tcxo_delay_ms=5.0,
            busy_timeout_ms=5000,
        )

        self.radio.open()

        # E22 modules drive the RF switch via DIO2 instead of dedicated TXEN/RXEN.
        self.radio.set_dio2_as_rf_switch_ctrl(True)

        # DC-DC regulator for better efficiency
        self.radio.set_regulator_mode(vd.REGULATOR_DC_DC)

        # Carrier frequency
        self.radio.set_frequency(self.frequency)

        # TX power (SX1262 variant)
        self.radio.set_tx_power(self.txpower, vd.TX_POWER_SX1262)

        # Boosted RX gain for best sensitivity
        self.radio.set_rx_gain(vd.RX_GAIN_BOOSTED)

        # Modulation
        ldro = self._should_use_ldro()
        self.radio.set_lora_modulation(self.sf, self.bandwidth, self.cr, ldro)

        # Packet params: explicit header, 8-symbol preamble, max 255-byte payload,
        # CRC on, no IQ invert.
        self.radio.set_lora_packet(
            vd.HEADER_EXPLICIT,
            8,
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

            self.radio.set_dio2_as_rf_switch_ctrl(True)
            self.radio.set_regulator_mode(vd.REGULATOR_DC_DC)
            self.radio.set_frequency(self.frequency)
            self.radio.set_tx_power(self.txpower, vd.TX_POWER_SX1262)
            self.radio.set_rx_gain(vd.RX_GAIN_BOOSTED)

            ldro = self._should_use_ldro()
            self.radio.set_lora_modulation(self.sf, self.bandwidth, self.cr, ldro)

            self.radio.set_lora_packet(
                vd.HEADER_EXPLICIT,
                8,
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
        preamble_len = 8
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

        while not self._stop_event.is_set():
            # ---- 1. Block briefly on the radio's IRQ edge ----
            irq = None
            try:
                irq = self.radio.wait_irq_done(0.1)
                self._consecutive_spi_failures = 0
            except Exception as e:
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

        RNS.log(str(self) + " radio thread exiting", RNS.LOG_VERBOSE)

    def _handle_irq(self, irq):
        """Dispatch an SX126x IRQ status word to the appropriate handler."""
        vd = self.vd
        if irq & vd.IRQ_RX_DONE:
            self._handle_rx_done()
        if irq & vd.IRQ_TX_DONE:
            self._handle_tx_done()
        if irq & vd.IRQ_TIMEOUT:
            RNS.log(str(self) + " radio reported timeout IRQ", RNS.LOG_DEBUG)
            try:
                self.radio.clear_irq_status(vd.IRQ_TIMEOUT)
            except Exception:
                pass
        if irq & vd.IRQ_CRC_ERR:
            RNS.log(str(self) + " CRC error on received frame", RNS.LOG_DEBUG)
            try:
                self.radio.clear_irq_status(vd.IRQ_CRC_ERR)
            except Exception:
                pass
        if irq & vd.IRQ_HEADER_ERR:
            RNS.log(str(self) + " header error on received frame", RNS.LOG_DEBUG)
            try:
                self.radio.clear_irq_status(vd.IRQ_HEADER_ERR)
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

        # Chip-internal TX timeout, encoded in 15.625us units. 1.2x TOA
        # so the chip times out just after we would.
        chip_timeout_units = int(toa * 64000.0 * 1.2) & 0x00FFFFFF

        try:
            self.radio.set_standby(vd.STANDBY_RC)
            # Make sure the chip knows the current max payload length.
            self.radio.set_lora_packet(
                vd.HEADER_EXPLICIT,
                8,
                SX126xInterface.LORA_MAX_PAYLOAD,
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