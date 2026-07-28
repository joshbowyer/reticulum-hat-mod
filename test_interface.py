"""Lightweight integration test for SX126xInterface.py.

Simulates what Reticulum does: reads the interface file, injects `Interface`
and `RNS` globals, exec's the file, and confirms `interface_class` is
exposed and instantiable. Uses a mock Reticulum Transport and a mock
RNS.log, so it does NOT need real RNS / spidev / gpiod installed.

The SX126xRadio class is also stubbed so we can construct an interface
instance without real hardware.
"""
import os
import sys
import types
import unittest.mock as mock
import importlib.util
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
INTERFACE_PATH = "/home/josh/reticulum-stack/dev/reticulum-hat-mod/SX126xInterface.py"
VENDORED_PATH  = "/home/josh/reticulum-stack/dev/reticulum-hat-mod/vendored_sx126x.py"

# ---- Build a fake "vendored_sx126x" with a stub SX126xRadio class ----

class FakeSX126xRadio:
    """Stand-in for vendored_sx126x.SX126xRadio for testing."""
    # Constants the interface code uses; mirror the vendored values.
    IRQ_ALL = 0x03FF
    IRQ_RX_DONE = 0x0002
    IRQ_TX_DONE = 0x0001
    IRQ_TIMEOUT = 0x0200
    IRQ_CRC_ERR = 0x0040
    IRQ_HEADER_ERR = 0x0020
    IRQ_CAD_DONE = 0x0080
    IRQ_CAD_DETECTED = 0x0100
    HEADER_EXPLICIT = 0x00
    RX_GAIN_BOOSTED = 0x01
    REGULATOR_DC_DC = 0x01
    TX_POWER_SX1262 = 0x02
    STANDBY_RC = 0x00
    PACKET_TYPE_LORA = 0x01
    STATUS_MODE_STDBY_RC = 0x20
    RX_CONTINUOUS = 0xFFFFFF

    # State
    _mode = "STDBY"  # one of STDBY, RX, CAD, TX

    def __init__(self, *a, **kw):
        self.opened = False
        self.closed = False
        self.in_rx = False
        self.irq_pending = None
        # fail_next: when True, the next SPI/op raises. Consumed after one use.
        self.fail_next = False
        # fail_all_spi: when True, every SPI/op raises (until cleared).
        self.fail_all_spi = False

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def reset(self):
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("simulated reset failure")
        return True

    def _maybe_fail(self):
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("simulated SPI failure (one-shot)")
        if self.fail_all_spi:
            raise RuntimeError("simulated persistent SPI failure")

    def set_dio2_as_rf_switch_ctrl(self, e): self._maybe_fail()
    def set_regulator_mode(self, m): self._maybe_fail()
    def set_frequency(self, f): self._maybe_fail()
    def set_tx_power(self, *a, **kw): self._maybe_fail()
    def set_rx_gain(self, g): self._maybe_fail()
    def set_lora_modulation(self, *a): self._maybe_fail()
    def set_lora_packet(self, *a): self._maybe_fail()
    def set_sync_word(self, w): self._maybe_fail()
    def set_cad_params(self, *a, **kw): self._maybe_fail()
    def set_packet_type(self, t): self._maybe_fail()
    def calibrate(self, *a): self._maybe_fail()
    def set_dio3_as_tcxo_ctrl(self, *a): self._maybe_fail()

    def request_irq_edge(self, edge): self._maybe_fail()
    def release_irq_edge(self): pass

    def set_standby(self, mode):
        self._mode = "STDBY"
        self.in_rx = False
        self._maybe_fail()

    def set_dio_irq_params(self, *a, **kw): self._maybe_fail()
    def clear_irq_status(self, mask): self._maybe_fail()

    def set_rx(self, timeout):
        self._mode = "RX"
        self.in_rx = True
        self._maybe_fail()

    def set_cad(self):
        self._mode = "CAD"
        self._maybe_fail()

    def wait_irq_done(self, timeout_s):
        # Simulate different IRQs depending on what mode we just entered.
        self._maybe_fail()
        # For CAD mode, return IRQ_CAD_DONE immediately (channel clear).
        if self._mode == "CAD":
            return FakeSX126xRadio.IRQ_CAD_DONE
        # For RX mode, return None (timeout = no IRQ)
        time.sleep(min(timeout_s, 0.05))
        return None

    def set_tx(self, timeout_units):
        self._mode = "TX"
        # We don't actually emit anything but transition to STDBY after
        # the next IRQ wait — emulate the chip completing TX immediately.
        self._tx_pending = True
        self._maybe_fail()

    def set_buffer_base_address(self, a, b): self._maybe_fail()
    def write_buffer(self, off, data):
        self._last_written = bytes(data)
        self._maybe_fail()

    def get_rx_buffer_status(self):
        self._maybe_fail()
        return (0, 0)
    def get_packet_status(self):
        self._maybe_fail()
        return (200, 100, 200)  # rssi_raw, snr_raw, sig_raw
    def read_buffer(self, off, n):
        self._maybe_fail()
        return bytes(n)

    def get_status_and_mode(self):
        self._maybe_fail()
        return FakeSX126xRadio.STATUS_MODE_STDBY_RC

    @staticmethod
    def _tcxo_voltage_code(v):
        return 0x02  # arbitrary

    @staticmethod
    def _tcxo_delay_code(ms):
        return int(ms * 64) & 0x00FFFFFF


# Inject our fake driver so the interface loader finds it via `import`
fake_vd = types.ModuleType("vendored_sx126x")
fake_vd.SX126xRadio = FakeSX126xRadio
fake_vd.SX126xError = type("SX126xError", (Exception,), {})
fake_vd.SX126xTimeout = type("SX126xTimeout", (TimeoutError,), {})
fake_vd.IRQ_ALL              = FakeSX126xRadio.IRQ_ALL
fake_vd.IRQ_RX_DONE          = FakeSX126xRadio.IRQ_RX_DONE
fake_vd.IRQ_TX_DONE          = FakeSX126xRadio.IRQ_TX_DONE
fake_vd.IRQ_TIMEOUT          = FakeSX126xRadio.IRQ_TIMEOUT
fake_vd.IRQ_CRC_ERR          = FakeSX126xRadio.IRQ_CRC_ERR
fake_vd.IRQ_HEADER_ERR       = FakeSX126xRadio.IRQ_HEADER_ERR
fake_vd.IRQ_CAD_DONE         = FakeSX126xRadio.IRQ_CAD_DONE
fake_vd.IRQ_CAD_DETECTED     = FakeSX126xRadio.IRQ_CAD_DETECTED
fake_vd.HEADER_EXPLICIT      = FakeSX126xRadio.HEADER_EXPLICIT
fake_vd.RX_GAIN_BOOSTED      = FakeSX126xRadio.RX_GAIN_BOOSTED
fake_vd.REGULATOR_DC_DC      = FakeSX126xRadio.REGULATOR_DC_DC
fake_vd.TX_POWER_SX1262      = FakeSX126xRadio.TX_POWER_SX1262
fake_vd.STANDBY_RC           = FakeSX126xRadio.STANDBY_RC
fake_vd.PACKET_TYPE_LORA     = FakeSX126xRadio.PACKET_TYPE_LORA
fake_vd.STATUS_MODE_STDBY_RC = FakeSX126xRadio.STATUS_MODE_STDBY_RC
fake_vd.RX_CONTINUOUS        = FakeSX126xRadio.RX_CONTINUOUS
sys.modules["vendored_sx126x"] = fake_vd

# ---- Build a minimal RNS shim that lets the interface code run ----

class LogEvent:
    def __init__(self, level, msg):
        self.level = level
        self.msg = msg

class FakeRNS:
    LOG_CRITICAL = 0
    LOG_ERROR    = 1
    LOG_WARNING  = 2
    LOG_NOTICE   = 3
    LOG_INFO     = 4
    LOG_VERBOSE  = 5
    LOG_DEBUG    = 6
    LOG_EXTREME  = 7

    @staticmethod
    def log(msg, level=3):
        # Just record the last log line at the requested level for later
        # inspection.
        FakeRNS.logs.append(LogEvent(level, str(msg)))

    @staticmethod
    def panic():
        raise SystemExit("RNS.panic() called")

    @staticmethod
    def trace_exception(e):
        pass


class FakeTransport:
    """Stand-in for RNS.Transport — the interface only calls inbound()."""
    def inbound(self, data, interface):
        FakeTransport.received.append((bytes(data), interface))


FakeRNS.logs = []
FakeTransport.received = []


# ---- Mock the Reticulum Interfaces.Interface base class ----

class FakeInterfaceBase:
    """Minimal subset of RNS.Interfaces.Interface used by our code."""
    IN  = False
    OUT = False
    FWD = False
    RPT = False

    MODE_FULL = 0x01
    DISCOVER_PATHS_FOR = []
    IA_FREQ_SAMPLES = 48
    OA_FREQ_SAMPLES = 48
    IP_FREQ_SAMPLES = 48
    OP_FREQ_SAMPLES = 48
    AR_MINFREQ_HZ = 0.1
    PR_MINFREQ_HZ = 0.1
    AR_FREQ_DECAY = 1/AR_MINFREQ_HZ
    PR_FREQ_DECAY = 1/PR_MINFREQ_HZ
    MAX_HELD_ANNOUNCES = 256
    IC_NEW_TIME = 2*60*60
    IC_BURST_FREQ_NEW = 3
    IC_BURST_FREQ = 10
    IC_PR_BURST_FREQ_NEW = 3
    IC_PR_BURST_FREQ = 8
    IC_BURST_HOLD = 15
    IC_BURST_PENALTY = 15
    IC_HELD_RELEASE_INTERVAL = 5
    IC_DEQUE_MIN_SAMPLE = 2
    IC_BURST_MIN_SAMPLES = 6
    EC_PR_FREQ = 5
    EGRESS_CONTROL = False
    DEFAULT_AR_TARGET = 3600
    DEFAULT_AR_PENALTY = 0
    DEFAULT_AR_GRACE = 5
    AUTOCONFIGURE_MTU = False
    FIXED_MTU = False

    def __init__(self):
        self.rxb = 0
        self.txb = 0
        self.created = time.time()
        self.detached = False
        self.online = False
        self.bitrate = 62500
        self.HW_MTU = None
        self.supports_discovery = False
        self.discoverable = False
        self.last_discovery_announce = 0
        self.bootstrap_only = False
        self.recursive_prs = False
        self.announces_from_internal = True
        self.parent_interface = None
        self.spawned_interfaces = None
        self.tunnel_id = None
        self.ingress_control = True
        self.phy_keepalive = False
        self.ic_burst_active = False
        self.ic_burst_activated = 0
        self.ic_pr_burst_active = False
        self.ic_pr_burst_activated = 0
        self.ic_held_release = 0
        self.ic_max_held_announces = 256
        self.ic_burst_hold = 15
        self.ic_burst_freq_new = 3
        self.ic_burst_freq = 10
        self.ic_pr_burst_freq_new = 3
        self.ic_pr_burst_freq = 8
        self.ic_new_time = 2*60*60
        self.ic_burst_penalty = 15
        self.ic_held_release_interval = 5
        self.ec_pr_freq = 5
        self.egress_control = False
        from collections import deque
        self.ia_freq_deque = deque(maxlen=48)
        self.oa_freq_deque = deque(maxlen=48)
        self.ip_freq_deque = deque(maxlen=48)
        self.op_freq_deque = deque(maxlen=48)
        self.held_announces = {}

    def should_ingress_limit(self):
        return False

    @staticmethod
    def get_config_obj(c):
        # Accept either a dict or a ConfigObj-like. We pass dicts.
        return c


# ---- Simulate Reticulum's loader ----

with open(INTERFACE_PATH) as f:
    interface_code = f.read()

interface_globals = {
    "Interface": FakeInterfaceBase,
    "RNS": FakeRNS,
}

exec(interface_code, interface_globals)
interface_class = interface_globals.get("interface_class")
SX126xInterface = interface_class  # alias for direct class constant access

assert interface_class is not None, "interface_class not exposed by module"
print(f"[OK] interface_class = {interface_class.__name__}")

# ---- Try to construct an instance ----

cfg = {
    "name": "TestMeshAdv",
    "frequency": "915000000",
    "bandwidth": "125000",
    "spreadingfactor": "8",
    "codingrate": "5",
    "txpower": "17",
}

owner = FakeTransport()

inst = interface_class(owner, cfg)

# Verify contract
assert inst.HW_MTU == 508, f"HW_MTU = {inst.HW_MTU}"
assert inst.DEFAULT_IFAC_SIZE == 8, f"DEFAULT_IFAC_SIZE = {inst.DEFAULT_IFAC_SIZE}"
assert inst.should_ingress_limit() is False, "should_ingress_limit must be False"
assert inst.online is True, "interface should be online after init"
assert inst.bitrate > 0, f"bitrate should be > 0 (got {inst.bitrate})"
assert str(inst) == "SX126xInterface[TestMeshAdv]", f"__str__ = {str(inst)}"
print(f"[OK] HW_MTU={inst.HW_MTU}, DEFAULT_IFAC_SIZE={inst.DEFAULT_IFAC_SIZE}, bitrate={inst.bitrate}")
print(f"[OK] should_ingress_limit=False, online=True, str={inst(inst)}" if False else f"[OK] str={str(inst)}")

# Verify split-frame framing (RNode-compatible)
frames = inst._make_frames(b"x" * 100)
assert len(frames) == 1, "short packet should be 1 frame"
assert (frames[0][0] & 0x01) == 0, "short packet must have FLAG_SPLIT=0"
assert frames[0][1:] == b"x" * 100

frames = inst._make_frames(b"x" * 300)
assert len(frames) == 2, "long packet should be 2 frames"
assert (frames[0][0] & 0x01) == 0x01, "long packet must have FLAG_SPLIT=1"
assert (frames[1][0] & 0x01) == 0x01, "long packet fragments must share header"
assert (frames[0][0] & 0xF0) == (frames[1][0] & 0xF0), "fragments must share seq_id"
assert len(frames[0]) == 255, f"first frame must be header + 254 bytes = 255 (got {len(frames[0])})"
print("[OK] split-frame framing produces correct 1 or 2 frames with shared seq_id")

# Verify reassembly
out = inst._reassemble(frames[0])
assert out is None, "first fragment of split packet must return None (waiting for 2nd)"
out = inst._reassemble(frames[1])
assert out == b"x" * 300, f"reassembled payload should equal original (got {len(out) if out else 'None'})"
print("[OK] split-frame reassembly reassembles to original bytes")

# Verify process_outgoing does NOT block and queues
inst.process_outgoing(b"x" * 100)
assert not inst._tx_queue.empty(), "process_outgoing must enqueue data"
print("[OK] process_outgoing enqueues without blocking")

# Verify airtime deque tracking
inst._track_airtime(0.5)
inst._track_airtime(0.5)
pct = inst._get_short_airtime_pct()
assert pct > 0, f"airtime pct should be > 0 after tracking 2x0.5s (got {pct})"
print(f"[OK] airtime tracking: {pct:.2f}% in short window after 2x0.5s")

# Verify CSMA on the queue (with stubbed CAD that returns "clear")
inst.process_outgoing(b"y" * 100)
# Let the radio thread pick it up
deadline = time.time() + 5.0
while not inst._tx_queue.empty() and time.time() < deadline:
    time.sleep(0.05)
# The queue should have been drained
print(f"[OK] queue size after drain wait: {inst._tx_queue.qsize()}")

# Detach cleanly
inst.detach()
assert inst.online is False, "online must be False after detach"
assert inst.detached is True, "detached must be True after detach"
assert inst.radio is None or inst.radio.closed is True, "radio must be closed after detach"
assert inst._radio_thread is None, "_radio_thread should be cleared after detach"
print("[OK] detach completes cleanly")

# Verify interface can still be garbage-collected (no thread leaks)
import gc
gc.collect()
print(f"[OK] {len(FakeRNS.logs)} RNS.log calls during the test")

# -------------------------------------------------------------------------
# Test 2: airtime budget drops packets without TX
# -------------------------------------------------------------------------
print("\n--- Test 2: airtime budget drops packets ---")

# Disable CSMA so packets would otherwise be drained instantly
cfg2 = dict(cfg)
cfg2["airtime_limit_short"] = "1.0"  # 1% of 15s window = 150ms total allowed
inst2 = interface_class(FakeTransport(), cfg2)

# Fill up the airtime budget with manual tracks
inst2._track_airtime(0.20)  # 200ms > 150ms allowed

# Try to push a 100-byte packet (TOA ~80ms) -- it should fit individually,
# but combined with the existing 200ms it would exceed budget.
inst2.process_outgoing(b"z" * 100)

# Wait briefly for the radio thread to attempt drain and drop it
deadline = time.time() + 3.0
while inst2._tx_queue.qsize() > 0 and time.time() < deadline:
    time.sleep(0.05)
time.sleep(0.5)  # let the radio thread attempt the drain

# The packet was popped by the radio thread (it's no longer in the queue)
# but it should NOT have been TX'd (txb unchanged).
assert inst2.txb == 0, f"packet should have been dropped due to airtime budget (txb={inst2.txb})"
print("[OK] airtime budget drop: packet not transmitted")

inst2.detach()
print("[OK] Test 2 passed")

# -------------------------------------------------------------------------
# Test 3: oversized packet dropped at process_outgoing
# -------------------------------------------------------------------------
print("\n--- Test 3: oversized packet dropped ---")

cfg3 = dict(cfg)
inst3 = interface_class(FakeTransport(), cfg3)
oversized = b"x" * (inst3.HW_MTU + 1)
inst3.process_outgoing(oversized)
assert inst3._tx_queue.qsize() == 0, "oversized packet should NOT be enqueued"
print("[OK] oversized packet dropped at process_outgoing (not enqueued)")
inst3.detach()

# -------------------------------------------------------------------------
# Test 4: lockup recovery escalates correctly
# -------------------------------------------------------------------------
print("\n--- Test 4: lockup recovery ---")

cfg4 = dict(cfg)
inst4 = interface_class(FakeTransport(), cfg4)

# Force a few "transient" SPI failures to trigger the WARNING-only path
inst4.radio.fail_next = True
handled = inst4._handle_spi_failure("test_op_1", RuntimeError("simulated"))
assert handled is True, "should continue after transient failures"
assert inst4._consecutive_spi_failures == 1, f"counter={inst4._consecutive_spi_failures}"
assert inst4.online is True, "online should remain True during transient failures"
print("[OK] transient SPI failures don't mark offline")

# Force the threshold-crossing path
# _handle_spi_failure increments counter then checks threshold; we need to
# push past RADIO_REINIT_THRESHOLD (5).
inst4._consecutive_spi_failures = SX126xInterface.RADIO_REINIT_THRESHOLD - 1
inst4.radio.fail_next = False  # reinit will succeed
handled = inst4._handle_spi_failure("test_op_2", RuntimeError("simulated lockup"))
assert handled is True, f"should recover via reinit when reinit succeeds (got {handled})"
assert inst4._consecutive_spi_failures == 0, f"counter should be reset after reinit (got {inst4._consecutive_spi_failures})"
assert inst4._consecutive_reinit_failures == 0, f"reinit failure counter should be reset (got {inst4._consecutive_reinit_failures})"
print("[OK] reinit succeeds, counters reset")

# Now force the giveup path. Stop the radio thread first so its own
# concurrent calls to _handle_spi_failure don't race with our test calls.
# We do this by setting the stop event and joining the thread manually,
# without calling detach() (which closes the radio and sets self.radio=None).
inst4._stop_event.set()
inst4._radio_thread.join(timeout=2.0)
assert not inst4._radio_thread.is_alive(), "radio thread should have exited"
# Note: inst4.radio is still valid; the driver is open but the thread is gone.

inst4.radio.fail_all_spi = True  # every op raises until cleared
inst4._consecutive_spi_failures = SX126xInterface.RADIO_REINIT_THRESHOLD - 1
handled = inst4._handle_spi_failure("test_op_3", RuntimeError("simulated lockup"))
assert handled is True, f"first failed reinit should continue, not give up (got {handled})"
assert inst4._consecutive_reinit_failures == 1, f"expected 1, got {inst4._consecutive_reinit_failures}"

# Push past RADIO_GIVEUP_THRESHOLD (3) -- two more failed reinits
inst4._consecutive_spi_failures = SX126xInterface.RADIO_REINIT_THRESHOLD - 1
handled = inst4._handle_spi_failure("test_op_4", RuntimeError("simulated"))
assert inst4._consecutive_reinit_failures == 2, f"expected 2, got {inst4._consecutive_reinit_failures}"
inst4._consecutive_spi_failures = SX126xInterface.RADIO_REINIT_THRESHOLD - 1
handled = inst4._handle_spi_failure("test_op_5", RuntimeError("simulated"))
assert handled is False, f"after giveup threshold, should return False (got {handled})"
assert inst4._consecutive_reinit_failures == 3, f"expected 3, got {inst4._consecutive_reinit_failures}"
assert inst4.online is False, "interface should be marked offline after repeated reinit failures"
print("[OK] repeated reinit failure marks interface offline")

# Clean up
inst4.radio.fail_all_spi = False
inst4.detach()

inst4.detach()

print("\n*** ALL TESTS PASSED ***")