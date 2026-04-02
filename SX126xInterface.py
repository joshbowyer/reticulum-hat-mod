##########################################################
# SX126xInterface.py                                     #
#                                                        #
# A Reticulum custom interface that drives SX1262/SX1268 #
# LoRa chips directly over SPI on Linux SBCs (Raspberry  #
# Pi, Orange Pi, etc). No separate RNode MCU required.   #
#                                                        #
# Implements:                                            #
#   - Direct SPI control via LoRaRF-Python library       #
#   - CSMA/CA collision avoidance (p-persistent)         #
#   - RNode-compatible split-packet framing for the      #
#     full Reticulum 500-byte MTU                        #
#   - Configurable pin mapping for various Pi HATs       #
#                                                        #
# Default pin config is for the MeshAdv Pi HAT v1.1:     #
#   CS=21, IRQ=16, Busy=20, Reset=18, TXen=13, RXen=12  #
#                                                        #
# Place this file in ~/.reticulum/interfaces/ and add    #
# an interface config block to ~/.reticulum/config       #
#                                                        #
# Example config:                                        #
#                                                        #
#   [[MeshAdv LoRa]]                                     #
#     type = SX126xInterface                             #
#     interface_enabled = True                            #
#     frequency = 915000000                              #
#     bandwidth = 125000                                 #
#     spreadingfactor = 8                                #
#     codingrate = 5                                     #
#     txpower = 22                                       #
#     # Pin config (MeshAdv Pi HAT defaults)             #
#     spi_bus = 0                                        #
#     spi_cs = 0                                         #
#     pin_cs = 21                                        #
#     pin_irq = 16                                       #
#     pin_busy = 20                                      #
#     pin_reset = 18                                     #
#     pin_txen = 13                                      #
#     pin_rxen = 12                                      #
#     pin_dio = 16                                       #
#     dio3_tcxo_voltage = 1.8                            #
#     # CSMA config (optional)                           #
#     csma_p = 0.1                                       #
#     csma_slot_ms = 50                                  #
#     csma_max_backoff = 5                               #
#                                                        #
# License: MIT                                           #
##########################################################

from RNS.Interfaces.Interface import Interface
import threading
import time
import math
import os
import random
import RNS


class SX126xInterface(Interface):
    MAX_CHUNK       = 32768
    DEFAULT_IFAC_SIZE = 8

    FREQ_MIN = 137000000
    FREQ_MAX = 1020000000

    # SX1262 maximum single LoRa frame payload
    LORA_MAX_PAYLOAD = 255
    # We prepend a 1-byte split-frame header, so usable payload per frame
    FRAME_PAYLOAD_MAX = 254

    # Split-packet framing flags (RNode-compatible)
    FLAG_SPLIT = 0x01

    RSSI_OFFSET = 157

    def __init__(self, owner, configuration):
        super().__init__()

        c = Interface.get_config_obj(configuration)
        name = c["name"]

        # LoRa radio parameters
        frequency = int(c["frequency"]) if "frequency" in c else 915000000
        bandwidth = int(c["bandwidth"]) if "bandwidth" in c else 125000
        txpower   = int(c["txpower"])   if "txpower" in c else 22
        sf        = int(c["spreadingfactor"]) if "spreadingfactor" in c else 8
        cr        = int(c["codingrate"]) if "codingrate" in c else 5

        # SPI bus configuration
        spi_bus   = int(c["spi_bus"])   if "spi_bus" in c else 0
        spi_cs    = int(c["spi_cs"])    if "spi_cs" in c else 0

        # GPIO pin configuration (BCM numbering)
        # MeshAdv Pi HAT v1.1 defaults
        pin_cs    = int(c["pin_cs"])    if "pin_cs" in c else 21
        pin_irq   = int(c["pin_irq"])   if "pin_irq" in c else 16
        pin_busy  = int(c["pin_busy"])  if "pin_busy" in c else 20
        pin_reset = int(c["pin_reset"]) if "pin_reset" in c else 18
        pin_txen  = int(c["pin_txen"])  if "pin_txen" in c else 13
        pin_rxen  = int(c["pin_rxen"])  if "pin_rxen" in c else 12

        # DIO3 TCXO voltage (set to 0 or None to disable)
        dio3_tcxo = float(c["dio3_tcxo_voltage"]) if "dio3_tcxo_voltage" in c else 1.8

        # Sync word: 0x12 = private/RNode, 0x34 = public/LoRaWAN
        sync_word = int(c["sync_word"], 0) if "sync_word" in c else 0x12

        # CSMA/CA parameters
        self.csma_p          = float(c["csma_p"])          if "csma_p" in c else 0.1
        self.csma_slot_ms    = float(c["csma_slot_ms"])    if "csma_slot_ms" in c else 50.0
        self.csma_max_backoff = int(c["csma_max_backoff"]) if "csma_max_backoff" in c else 5

        # Airtime limit (optional, percent 0-100)
        self.st_alock = float(c["airtime_limit_short"]) if "airtime_limit_short" in c else None
        self.lt_alock = float(c["airtime_limit_long"])  if "airtime_limit_long" in c else None

        self.HW_MTU = 508

        self.owner       = owner
        self.name        = name
        self.online      = False
        self.detached    = False

        self.frequency   = frequency
        self.bandwidth   = bandwidth
        self.txpower     = txpower
        self.sf          = sf
        self.cr          = cr
        self.sync_word   = sync_word
        self.spi_bus     = spi_bus
        self.spi_cs      = spi_cs
        self.pin_cs      = pin_cs
        self.pin_irq     = pin_irq
        self.pin_busy    = pin_busy
        self.pin_reset   = pin_reset
        self.pin_txen    = pin_txen
        self.pin_rxen    = pin_rxen
        self.dio3_tcxo   = dio3_tcxo

        self.bitrate     = 0
        self.r_stat_rssi = None
        self.r_stat_snr  = None

        self.packet_queue    = []
        self.tx_lock         = threading.Lock()
        self.interface_ready = True
        self.announce_rate_target = None

        # Airtime tracking
        self._airtime_short_window = 15.0   # seconds
        self._airtime_long_window  = 60.0 * 60.0  # 1 hour
        self._airtime_short_sum    = 0.0
        self._airtime_long_sum     = 0.0
        self._airtime_short_start  = time.time()
        self._airtime_long_start   = time.time()

        # Split-packet reassembly state
        self._rx_fragments = {}  # seq_id -> (timestamp, data)
        self._frag_timeout = 10.0  # seconds to wait for second fragment

        # Validate configuration
        validcfg = True
        if frequency < SX126xInterface.FREQ_MIN or frequency > SX126xInterface.FREQ_MAX:
            RNS.log(f"Invalid frequency configured for {self}", RNS.LOG_ERROR)
            validcfg = False
        if txpower < -9 or txpower > 22:
            RNS.log(f"Invalid TX power configured for {self}", RNS.LOG_ERROR)
            validcfg = False
        if bandwidth not in [7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000]:
            RNS.log(f"Invalid bandwidth configured for {self}", RNS.LOG_ERROR)
            validcfg = False
        if sf < 5 or sf > 12:
            RNS.log(f"Invalid spreading factor configured for {self}", RNS.LOG_ERROR)
            validcfg = False
        if cr < 5 or cr > 8:
            RNS.log(f"Invalid coding rate configured for {self}", RNS.LOG_ERROR)
            validcfg = False

        if not validcfg:
            raise ValueError(f"The configuration for {self} contains errors, interface is offline")

        try:
            self._init_radio()
            self._update_bitrate()
            self.online = True
            RNS.log(f"{self} is now online", RNS.LOG_NOTICE)
            RNS.log(f"  Frequency : {self.frequency/1e6} MHz", RNS.LOG_VERBOSE)
            RNS.log(f"  Bandwidth : {self.bandwidth/1e3} kHz", RNS.LOG_VERBOSE)
            RNS.log(f"  TX Power  : {self.txpower} dBm", RNS.LOG_VERBOSE)
            RNS.log(f"  SF        : {self.sf}", RNS.LOG_VERBOSE)
            RNS.log(f"  CR        : 4/{self.cr}", RNS.LOG_VERBOSE)
            RNS.log(f"  On-air    : {self.bitrate/1e3:.2f} kbps", RNS.LOG_VERBOSE)

            # Start the receive loop
            self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._rx_thread.start()

        except Exception as e:
            RNS.log(f"Could not initialise SX126x radio for {self}: {e}", RNS.LOG_ERROR)
            raise

    def _init_radio(self):
        """Initialise the SX1262/SX1268 radio chip over SPI."""
        try:
            from LoRaRF import SX126x
        except ImportError:
            RNS.log("The SX126xInterface requires the LoRaRF library.", RNS.LOG_CRITICAL)
            RNS.log("Install with: pip install LoRaRF", RNS.LOG_CRITICAL)
            RNS.panic()

        self.radio = SX126x()

        RNS.log(f"{self} Initialising SX126x on SPI bus {self.spi_bus} CS {self.spi_cs}", RNS.LOG_VERBOSE)

        if not self.radio.begin(
            bus=self.spi_bus,
            cs=self.spi_cs,
            reset=self.pin_reset,
            busy=self.pin_busy,
            irq=self.pin_irq,
            txen=self.pin_txen,
            rxen=self.pin_rxen
        ):
            raise IOError("SX126x radio not detected or failed to initialise")

        # Enable DIO3 as TCXO control if configured
        # The MeshAdv Pi HAT E22 modules use TCXO with DIO3
        if self.dio3_tcxo and self.dio3_tcxo > 0:
            # Map voltage to the closest DIO3 output constant
            tcxo_map = {
                1.6: SX126x.DIO3_OUTPUT_1_6,
                1.7: SX126x.DIO3_OUTPUT_1_7,
                1.8: SX126x.DIO3_OUTPUT_1_8,
                2.2: SX126x.DIO3_OUTPUT_2_2,
                2.4: SX126x.DIO3_OUTPUT_2_4,
                2.7: SX126x.DIO3_OUTPUT_2_7,
                3.0: SX126x.DIO3_OUTPUT_3_0,
                3.3: SX126x.DIO3_OUTPUT_3_3,
            }
            closest = min(tcxo_map.keys(), key=lambda v: abs(v - self.dio3_tcxo))
            self.radio.setDio3TcxoCtrl(tcxo_map[closest], SX126x.TCXO_DELAY_2_5)
            RNS.log(f"{self} TCXO enabled at {closest}V via DIO3", RNS.LOG_VERBOSE)

        # Enable DIO2 as RF switch control (used by E22 modules on MeshAdv)
        self.radio.setDio2RfSwitch(True)

        # Use DC-DC regulator for better efficiency
        self.radio.setRegulator(SX126x.REGULATOR_DC_DC)

        # Configure frequency
        self.radio.setFrequency(self.frequency)

        # Configure TX power (SX1262)
        self.radio.setTxPower(self.txpower, SX126x.TX_POWER_SX1262)

        # Set RX gain to boosted for best sensitivity
        self.radio.setRxGain(SX126x.RX_GAIN_BOOSTED)

        # Configure LoRa modulation parameters
        # LoRaRF expects bandwidth in Hz, SF, CR (5-8), LDRO auto
        bw_khz = self.bandwidth
        ldro = self._should_use_ldro()
        self.radio.setLoRaModulation(self.sf, bw_khz, self.cr, ldro)

        # Configure LoRa packet parameters
        # Explicit header, preamble length 8, max payload 255, CRC on, no IQ invert
        self.radio.setLoRaPacket(
            SX126x.HEADER_EXPLICIT,
            8,  # preamble symbols
            SX126xInterface.LORA_MAX_PAYLOAD,
            True,   # CRC enabled
            False   # no IQ invert
        )

        # Set sync word
        self.radio.setSyncWord(self.sync_word)

        RNS.log(f"{self} SX126x radio initialised successfully", RNS.LOG_VERBOSE)

    def _should_use_ldro(self):
        """Determine if Low Data Rate Optimization should be enabled."""
        # LDRO is needed when symbol time > 16ms
        symbol_time = (2**self.sf) / self.bandwidth
        return symbol_time > 0.016

    def _update_bitrate(self):
        """Calculate the on-air bitrate based on LoRa parameters."""
        # LoRa bitrate formula
        # Rb = SF * (BW / 2^SF) * CR_denom / CR_num
        # Where CR is expressed as 4/(cr), so the coding overhead is cr/4
        symbol_rate = self.bandwidth / (2**self.sf)
        bits_per_symbol = self.sf * (4.0 / self.cr)
        self.bitrate = symbol_rate * bits_per_symbol

    def _calculate_toa(self, payload_len):
        """Calculate time-on-air in seconds for a given payload length."""
        # Based on Semtech SX1262 datasheet LoRa modem formulas
        bw = self.bandwidth
        sf = self.sf
        cr = self.cr
        preamble_len = 8  # symbols
        has_crc = True
        explicit_header = True
        ldro = self._should_use_ldro()

        t_sym = (2**sf) / bw  # symbol duration in seconds

        # Preamble duration
        t_preamble = (preamble_len + 4.25) * t_sym

        # Payload symbol count
        de = 1 if ldro else 0
        ih = 0 if explicit_header else 1
        crc_bits = 16 if has_crc else 0

        num = max(8 * payload_len - 4 * sf + 28 + crc_bits - 20 * ih, 0)
        denom = 4 * (sf - 2 * de)
        n_payload = 8 + math.ceil(num / denom) * cr

        t_payload = n_payload * t_sym
        return t_preamble + t_payload

    #############################
    # CSMA/CA Implementation    #
    #############################

    def _channel_is_clear(self):
        """Check if the channel is clear using RSSI-based carrier sense."""
        try:
            # Read the current RSSI from the radio
            rssi = self.radio.packetRssi()
            # If RSSI is below the noise floor threshold, channel is clear
            # A typical threshold for clear channel is around -90 to -100 dBm
            return rssi < -90.0
        except Exception:
            # If we can't read RSSI, assume clear (best effort)
            return True

    def _csma_wait(self, frame_len):
        """
        Perform p-persistent CSMA/CA before transmitting.

        Uses exponential backoff with carrier sensing. This implements
        the same general approach as RNode firmware CSMA, adapted for
        direct SPI control from userspace.
        """
        slot_time = self.csma_slot_ms / 1000.0
        attempt = 0

        while attempt < self.csma_max_backoff * 10:
            if self._channel_is_clear():
                # Channel clear - transmit with probability p
                if random.random() < self.csma_p:
                    return  # Proceed with transmission
                else:
                    # Back off for one slot
                    time.sleep(slot_time)
            else:
                # Channel busy - exponential backoff
                backoff_exp = min(attempt // 2, self.csma_max_backoff)
                max_slots = 2**backoff_exp
                wait_slots = random.randint(0, max_slots)
                time.sleep(wait_slots * slot_time)

            attempt += 1

        # If we exhaust attempts, transmit anyway (best effort)
        RNS.log(f"{self} CSMA gave up after {attempt} attempts, transmitting", RNS.LOG_DEBUG)

    #############################
    # Split-Packet Framing      #
    #############################

    def _make_frames(self, data):
        """
        Split data into LoRa frames with RNode-compatible framing.

        Each frame gets a 1-byte header:
          - Upper nibble: 4-bit random sequence ID
          - Bit 0 (FLAG_SPLIT): 1 if packet is split across 2 frames

        Packets <= 254 bytes: single frame, FLAG_SPLIT=0
        Packets > 254 bytes:  split into 2 frames, FLAG_SPLIT=1,
                              same sequence ID in both
        """
        max_payload = SX126xInterface.FRAME_PAYLOAD_MAX
        frames = []

        if len(data) <= max_payload:
            # Single frame - no split needed
            seq_id = (random.randint(0, 15) << 4)
            header = seq_id & 0xF0  # FLAG_SPLIT = 0
            frames.append(bytes([header]) + data)
        else:
            # Split into exactly 2 frames
            seq_id = (random.randint(0, 15) << 4)
            header = (seq_id & 0xF0) | SX126xInterface.FLAG_SPLIT

            split_point = max_payload
            frame1 = bytes([header]) + data[:split_point]
            frame2 = bytes([header]) + data[split_point:]
            frames.append(frame1)
            frames.append(frame2)

        return frames

    def _reassemble(self, frame_data):
        """
        Reassemble received frames using the split-packet protocol.

        Returns the reassembled packet data, or None if waiting for
        the second fragment.
        """
        if len(frame_data) < 1:
            return None

        header = frame_data[0]
        seq_id = header & 0xF0
        is_split = bool(header & SX126xInterface.FLAG_SPLIT)
        payload = frame_data[1:]

        if not is_split:
            # Single frame packet - return immediately
            return payload

        # Split packet - need to reassemble
        now = time.time()

        # Clean up stale fragments
        stale = [k for k, (ts, _) in self._rx_fragments.items()
                 if now - ts > self._frag_timeout]
        for k in stale:
            del self._rx_fragments[k]

        if seq_id in self._rx_fragments:
            # Second fragment arrived - reassemble
            _, first_payload = self._rx_fragments.pop(seq_id)
            return first_payload + payload
        else:
            # First fragment - store and wait
            self._rx_fragments[seq_id] = (now, payload)
            return None

    #############################
    # TX / RX                   #
    #############################

    def _transmit_frame(self, frame):
        """Transmit a single LoRa frame via SPI."""
        self.radio.beginPacket()
        self.radio.put(frame)
        self.radio.endPacket()
        # Wait for TX to complete
        self.radio.wait(timeout=10)

        status = self.radio.status()
        from LoRaRF import SX126x
        if status == SX126x.STATUS_TX_DONE:
            return True
        else:
            RNS.log(f"{self} TX failed with status {status}", RNS.LOG_WARNING)
            return False

    def process_incoming(self, data):
        """Called when a complete packet has been received and reassembled."""
        self.rxb += len(data)
        self.owner.inbound(data, self)
        self.r_stat_rssi = None
        self.r_stat_snr = None

    def process_outgoing(self, data):
        """Called by Reticulum to send a packet out this interface."""
        if not self.online:
            return

        datalen = len(data)
        if datalen > self.HW_MTU:
            RNS.log(f"{self} Dropping oversized packet ({datalen} > {self.HW_MTU})", RNS.LOG_ERROR)
            return

        with self.tx_lock:
            try:
                frames = self._make_frames(data)

                for frame in frames:
                    # CSMA/CA before each frame
                    self._csma_wait(len(frame))

                    # Track airtime
                    toa = self._calculate_toa(len(frame))
                    self._track_airtime(toa)

                    # Check airtime limits
                    if self.st_alock and self._get_short_airtime_pct() > self.st_alock:
                        RNS.log(f"{self} Short-term airtime limit exceeded, queueing", RNS.LOG_DEBUG)
                        self.packet_queue.append(data)
                        return
                    if self.lt_alock and self._get_long_airtime_pct() > self.lt_alock:
                        RNS.log(f"{self} Long-term airtime limit exceeded, dropping", RNS.LOG_WARNING)
                        return

                    success = self._transmit_frame(frame)
                    if not success:
                        RNS.log(f"{self} Frame transmission failed", RNS.LOG_WARNING)
                        return

                    # Small inter-frame gap for split packets
                    if len(frames) > 1:
                        time.sleep(0.005)

                self.txb += datalen

                # Re-enter RX mode after TX
                self._start_rx()

            except Exception as e:
                RNS.log(f"{self} Error during transmission: {e}", RNS.LOG_ERROR)
                try:
                    self._start_rx()
                except Exception:
                    pass

    def _start_rx(self):
        """Put the radio into continuous receive mode."""
        from LoRaRF import SX126x
        self.radio.request(SX126x.RX_CONTINUOUS)

    def _receive_loop(self):
        """Background thread that continuously receives LoRa frames."""
        RNS.log(f"{self} Receive loop started", RNS.LOG_VERBOSE)

        from LoRaRF import SX126x

        # Enter continuous RX mode
        self._start_rx()

        while self.online and not self.detached:
            try:
                # Wait for a packet with a short timeout so we can
                # check the loop condition periodically
                if self.radio.wait(timeout=1):
                    status = self.radio.status()

                    if status == SX126x.STATUS_RX_DONE:
                        # Read the received data
                        payload_len = self.radio.available()
                        if payload_len > 0:
                            frame_data = self.radio.get(payload_len)

                            # Get signal quality
                            try:
                                self.r_stat_rssi = self.radio.packetRssi()
                                self.r_stat_snr = self.radio.snr()
                            except Exception:
                                pass

                            RNS.log(f"{self} Received frame ({len(frame_data)} bytes, "
                                    f"RSSI: {self.r_stat_rssi}, SNR: {self.r_stat_snr})",
                                    RNS.LOG_DEBUG)

                            # Reassemble split packets
                            packet = self._reassemble(frame_data)
                            if packet is not None:
                                self.process_incoming(packet)

                    elif status == SX126x.STATUS_CRC_ERR:
                        RNS.log(f"{self} CRC error on received frame", RNS.LOG_DEBUG)

                    elif status == SX126x.STATUS_HEADER_ERR:
                        RNS.log(f"{self} Header error on received frame", RNS.LOG_DEBUG)

                    # In continuous mode, the radio stays in RX
                    # but we may need to re-request after certain events
                    if status != SX126x.STATUS_RX_DONE:
                        # Small delay before retry
                        time.sleep(0.01)

            except Exception as e:
                RNS.log(f"{self} Error in receive loop: {e}", RNS.LOG_ERROR)
                time.sleep(1.0)
                try:
                    self._start_rx()
                except Exception:
                    pass

        RNS.log(f"{self} Receive loop ended", RNS.LOG_VERBOSE)

    #############################
    # Airtime Tracking          #
    #############################

    def _track_airtime(self, toa_seconds):
        """Track cumulative airtime for rate limiting."""
        now = time.time()
        self._airtime_short_sum += toa_seconds
        self._airtime_long_sum += toa_seconds

        # Reset windows if expired
        if now - self._airtime_short_start > self._airtime_short_window:
            self._airtime_short_sum = toa_seconds
            self._airtime_short_start = now
        if now - self._airtime_long_start > self._airtime_long_window:
            self._airtime_long_sum = toa_seconds
            self._airtime_long_start = now

    def _get_short_airtime_pct(self):
        """Get short-term airtime utilisation as a percentage."""
        elapsed = max(time.time() - self._airtime_short_start, 0.001)
        window = min(elapsed, self._airtime_short_window)
        return (self._airtime_short_sum / window) * 100.0

    def _get_long_airtime_pct(self):
        """Get long-term airtime utilisation as a percentage."""
        elapsed = max(time.time() - self._airtime_long_start, 0.001)
        window = min(elapsed, self._airtime_long_window)
        return (self._airtime_long_sum / window) * 100.0

    #############################
    # Interface Lifecycle       #
    #############################

    def detach(self):
        """Shut down the interface."""
        self.detached = True
        self.online = False
        try:
            self.radio.end()
        except Exception:
            pass
        RNS.log(f"{self} detached", RNS.LOG_NOTICE)

    def __str__(self):
        return f"SX126xInterface[{self.name}]"
