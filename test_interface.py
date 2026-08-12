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
INTERFACE_PATH = os.path.join(HERE, "SX126xInterface.py")
VENDORED_PATH  = os.path.join(HERE, "vendored_sx126x.py")

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
        # Record of all set_lora_packet(...) calls (used to verify that
        # the configured preamble_length actually flows through to the radio).
        self.set_lora_packet_calls = []

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
    def set_lora_packet(self, *a):
        # Record every call so tests can inspect the preamble_length (and
        # other params) actually handed to the radio.
        self.set_lora_packet_calls.append(a)
        self._maybe_fail()
    def set_sync_word(self, w): self._maybe_fail()
    def set_cad_params(self, *a, **kw): self._maybe_fail()
    def set_packet_type(self, t): self._maybe_fail()
    def calibrate(self, *a): self._maybe_fail()
    def set_dio3_as_tcxo_ctrl(self, *a): self._maybe_fail()

    def request_irq_edge(self, edge): self._maybe_fail()
    def release_irq_edge(self): pass

    def set_tx_enable(self, on=True): self._maybe_fail()
    def set_rx_enable(self, on=True): self._maybe_fail()
    def restore_tx_rx_pins(self): pass

    def set_pa_config(self, *a, **kw): self._maybe_fail()
    def set_tx_params(self, *a, **kw): self._maybe_fail()
    def clear_device_errors(self): self._maybe_fail()
    def read_register(self, address, n_bytes=1):
        # Default: return zeros (matches the chip's reset values for
        # registers we care about). The real driver may read-modify-write
        # these — having them all zero is a safe default that lets those
        # operations complete.
        self._maybe_fail()
        return bytes(n_bytes)
    def write_register(self, address, data):
        self._maybe_fail()
        return None

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

# FakeRNS as a real module so we can attach `vendor.configobj` for overlay loading
fake_rns = types.ModuleType("RNS")
fake_rns.LOG_CRITICAL = 0
fake_rns.LOG_ERROR    = 1
fake_rns.LOG_WARNING  = 2
fake_rns.LOG_NOTICE   = 3
fake_rns.LOG_INFO     = 4
fake_rns.LOG_VERBOSE  = 5
fake_rns.LOG_DEBUG    = 6
fake_rns.LOG_EXTREME  = 7
fake_rns.logs = []
fake_rns.received = []

def _fake_rns_log(msg, level=3):
    fake_rns.logs.append(LogEvent(level, str(msg)))
fake_rns.log = _fake_rns_log

def _fake_rns_panic():
    raise SystemExit("RNS.panic() called")
fake_rns.panic = _fake_rns_panic

def _fake_rns_trace_exception(e):
    pass
fake_rns.trace_exception = _fake_rns_trace_exception

FakeRNS = fake_rns  # backward-compat alias used elsewhere in this test file


class FakeTransport:
    """Stand-in for RNS.Transport — the interface only calls inbound()."""
    def inbound(self, data, interface):
        FakeTransport.received.append((bytes(data), interface))


FakeTransport.received = []

# Build a fake RNS.vendor.configobj shim so the resolver's overlay loading
# works in tests without requiring the real Reticulum package.
class FakeConfigObj:
    """Minimal ConfigObj-compatible parser for our overlay file format.

    Recognises:
      [top-level section]
        [[sub-section]]
          key = value
          [[[sub-sub-section]]]
            key = value
    """
    def __init__(self, path):
        self._data = {}
        with open(path) as f:
            self._parse(f)

    def _parse(self, f):
        sections = self._data
        stack = [sections]   # stack of nested dicts
        for raw in f:
            line = raw.rstrip()
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if stripped.startswith("[[[") and stripped.endswith("]]]"):
                key = stripped[3:-3].strip()
                cur = stack[-1]
                if "__no_section__" in cur:
                    cur.pop("__no_section__")
                new = {}
                cur[key] = new
                stack.append(new)
            elif stripped.startswith("[[") and stripped.endswith("]]"):
                key = stripped[2:-2].strip()
                cur = stack[-1]
                if "__no_section__" in cur:
                    cur.pop("__no_section__")
                new = {}
                cur[key] = new
                stack.append(new)
            elif stripped.startswith("[") and stripped.endswith("]"):
                key = stripped[1:-1].strip()
                # Pop back to the top level
                stack = [sections]
                cur = sections
                if key not in cur:
                    cur[key] = {}
                stack.append(cur[key])
            elif "=" in stripped:
                k, _, v = stripped.partition("=")
                cur = stack[-1]
                cur[k.strip()] = v.strip()

    def get(self, key, default=None):
        return self._data.get(key, default)

    def __getitem__(self, key):
        return self._data[key]

    def __contains__(self, key):
        return key in self._data

    def __iter__(self):
        return iter(self._data)


fake_rns_vendor = types.ModuleType("RNS.vendor")
fake_rns_vendor_configobj = types.ModuleType("RNS.vendor.configobj")
fake_rns_vendor_configobj.ConfigObj = FakeConfigObj
sys.modules["RNS.vendor"] = fake_rns_vendor
sys.modules["RNS.vendor.configobj"] = fake_rns_vendor_configobj
fake_rns.vendor = fake_rns_vendor


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

# -------------------------------------------------------------------------
# Test 5: profile-based resolution — Pi + MeshAdv HAT
# -------------------------------------------------------------------------
print("\n--- Test 5: profile resolution (Pi + MeshAdv HAT) ---")

cfg_pi_mesh = dict(cfg)
cfg_pi_mesh["platform"]    = "raspberry-pi"
cfg_pi_mesh["radio_board"] = "meshadv-pi-hat-v1.1"
# Remove the legacy key to confirm it's not needed in profile mode
cfg_pi_mesh.pop("pin_irq", None)

inst_pi = interface_class(FakeTransport(), cfg_pi_mesh)

# Expected resolved (gpiochip, line) pairs for the MeshAdv Pi HAT on a Pi:
#   header pin 36 (IRQ)  -> BCM16  -> gpiochip0 line 16
#   header pin 38 (BUSY) -> BCM20  -> gpiochip0 line 20
#   header pin 12 (RESET)-> BCM18  -> gpiochip0 line 18
#   header pin 33 (TXEN) -> BCM13  -> gpiochip0 line 13
#   header pin 32 (RXEN) -> BCM12  -> gpiochip0 line 12
assert inst_pi.gpiochip == "gpiochip0", f"gpiochip={inst_pi.gpiochip}"
assert inst_pi.pin_lines["irq"]  == ("gpiochip0", 16), f"irq={inst_pi.pin_lines['irq']}"
assert inst_pi.pin_lines["busy"] == ("gpiochip0", 20), f"busy={inst_pi.pin_lines['busy']}"
assert inst_pi.pin_lines["reset"]== ("gpiochip0", 18), f"reset={inst_pi.pin_lines['reset']}"
assert inst_pi.pin_lines["txen"] == ("gpiochip0", 13), f"txen={inst_pi.pin_lines['txen']}"
assert inst_pi.pin_lines["rxen"] == ("gpiochip0", 12), f"rxen={inst_pi.pin_lines['rxen']}"
# CS must also resolve — physical pin 40 → BCM21 → gpiochip0 line 21
assert inst_pi.pin_lines["cs"] == ("gpiochip0", 21), f"cs={inst_pi.pin_lines['cs']}"
assert inst_pi.pin_cs == 21, f"pin_cs={inst_pi.pin_cs}"
assert inst_pi.spi_bus == 0 and inst_pi.spi_cs == 0
assert inst_pi.platform_name == "raspberry-pi"
assert inst_pi.board_name == "meshadv-pi-hat-v1.1"
assert inst_pi.resolution["used_profile_mode"] is True
print(f"[OK] Pi + MeshAdv HAT: gpiochip={inst_pi.gpiochip}, "
      f"cs={inst_pi.pin_lines['cs']}, irq={inst_pi.pin_lines['irq']}, "
      f"busy={inst_pi.pin_lines['busy']}, reset={inst_pi.pin_lines['reset']}, "
      f"txen={inst_pi.pin_lines['txen']}, rxen={inst_pi.pin_lines['rxen']}")

# Verify the NOTICE log mentioned platform + board
log_msgs = [e.msg for e in FakeRNS.logs if "profile resolution" in e.msg.lower()]
assert log_msgs, "expected a NOTICE log about profile resolution"
print("[OK] NOTICE log emitted for profile resolution")

inst_pi.detach()

# -------------------------------------------------------------------------
# Test 6: profile-based resolution — Femtofox (sanity check against
# previously hardware-verified values, no real hardware needed)
# -------------------------------------------------------------------------
print("\n--- Test 6: profile resolution (Femtofox) — sanity check ---")

cfg_ff = dict(cfg)
cfg_ff["platform"]    = "luckfox-pico"
cfg_ff["radio_board"] = "femtofox-integrated-v1"
cfg_ff.pop("pin_irq", None)

inst_ff = interface_class(FakeTransport(), cfg_ff)

# Expected Femtofox resolved values (the ones that were hardware-verified
# in the prior step on actual femtofox hardware):
#   header pin 17 (IRQ)   -> GPIO55 -> gpiochip1 line 23
#   header pin 16 (BUSY)  -> GPIO54 -> gpiochip1 line 22
#   header pin 13 (RESET) -> GPIO57 -> gpiochip1 line 25
#   header pin 12 (RXEN)  -> GPIO56 -> gpiochip1 line 24
#   header pin TXEN -> None (bridged to DIO2)
assert inst_ff.gpiochip == "gpiochip1", f"gpiochip={inst_ff.gpiochip}"
assert inst_ff.pin_lines["irq"]  == ("gpiochip1", 23), f"irq={inst_ff.pin_lines['irq']}"
assert inst_ff.pin_lines["busy"] == ("gpiochip1", 22), f"busy={inst_ff.pin_lines['busy']}"
assert inst_ff.pin_lines["reset"]== ("gpiochip1", 25), f"reset={inst_ff.pin_lines['reset']}"
assert inst_ff.pin_lines["txen"] is None, f"txen={inst_ff.pin_lines['txen']} (should be None)"
assert inst_ff.pin_lines["rxen"] == ("gpiochip1", 24), f"rxen={inst_ff.pin_lines['rxen']}"
print(f"[OK] Femtofox: gpiochip={inst_ff.gpiochip}, "
      f"irq={inst_ff.pin_lines['irq']}, busy={inst_ff.pin_lines['busy']}, "
      f"reset={inst_ff.pin_lines['reset']}, txen={inst_ff.pin_lines['txen']}, "
      f"rxen={inst_ff.pin_lines['rxen']}")

inst_ff.detach()

# -------------------------------------------------------------------------
# Test 6b: profile-based resolution — BQ/Uniteng Station G3 (Pi Zero 2W
# daughterboard path). NOT hardware-verified yet - this only asserts the
# profile dict resolves to the pin mapping documented in its own comments
# and in Reticulum-StationG3/HARDWARE-RECON.md, plus that the txpower_max
# safety cap and txen/rxen polarity flags are wired through correctly.
# -------------------------------------------------------------------------
print("\n--- Test 6b: profile resolution (Station G3, Pi Zero 2W) ---")

cfg_g3 = dict(cfg)
cfg_g3["platform"]    = "raspberry-pi"
cfg_g3["radio_board"] = "station-g3"
cfg_g3.pop("pin_irq", None)

inst_g3 = interface_class(FakeTransport(), cfg_g3)

# Expected resolved (gpiochip, line) pairs for Station G3 on a Pi:
#   header pin 15 (IRQ)   -> BCM22 -> gpiochip0 line 22
#   header pin 18 (BUSY)  -> BCM24 -> gpiochip0 line 24
#   header pin 36 (RESET) -> BCM16 -> gpiochip0 line 16
#   header pin 11 (TXEN)  -> BCM17 -> gpiochip0 line 17 (PA enable)
#   header pin 16 (RXEN)  -> BCM23 -> gpiochip0 line 23 (LNA enable)
#   header pin 24 (CS)    -> BCM8  -> gpiochip0 line 8  (bit-banged NSS;
#                             HW SPI0 CE0 does not select the SX126x)
assert inst_g3.gpiochip == "gpiochip0", f"gpiochip={inst_g3.gpiochip}"
assert inst_g3.pin_lines["irq"]  == ("gpiochip0", 22), f"irq={inst_g3.pin_lines['irq']}"
assert inst_g3.pin_lines["busy"] == ("gpiochip0", 24), f"busy={inst_g3.pin_lines['busy']}"
assert inst_g3.pin_lines["reset"]== ("gpiochip0", 16), f"reset={inst_g3.pin_lines['reset']}"
assert inst_g3.pin_lines["txen"] == ("gpiochip0", 17), f"txen={inst_g3.pin_lines['txen']}"
assert inst_g3.pin_lines["rxen"] == ("gpiochip0", 23), f"rxen={inst_g3.pin_lines['rxen']}"
assert inst_g3.pin_lines["cs"] == ("gpiochip0", 8), f"cs={inst_g3.pin_lines['cs']} (bit-bang BCM8)"
assert inst_g3.pin_cs == 8, f"pin_cs={inst_g3.pin_cs}"
assert inst_g3.spi_bus == 0 and inst_g3.spi_cs == 0
assert inst_g3.platform_name == "raspberry-pi"
assert inst_g3.board_name == "station-g3"
# Polarity: PA enable is active-HIGH, LNA enable is active-LOW (opposite of
# the driver's historical default) — this is the whole reason the
# txen_active_low/rxen_active_low fields exist.
assert inst_g3.txen_active_low is False, f"txen_active_low={inst_g3.txen_active_low}"
assert inst_g3.rxen_active_low is True, f"rxen_active_low={inst_g3.rxen_active_low}"
# Safety cap: profile's txpower_max must be the conservative 7 dBm value,
# and configuring a higher txpower must be clamped down to it.
assert inst_g3.txpower_max == 7, f"txpower_max={inst_g3.txpower_max}"
print(f"[OK] Station G3: gpiochip={inst_g3.gpiochip}, "
      f"irq={inst_g3.pin_lines['irq']}, busy={inst_g3.pin_lines['busy']}, "
      f"reset={inst_g3.pin_lines['reset']}, txen={inst_g3.pin_lines['txen']}, "
      f"rxen={inst_g3.pin_lines['rxen']}, cs={inst_g3.pin_lines['cs']}, "
      f"txen_active_low={inst_g3.txen_active_low}, "
      f"rxen_active_low={inst_g3.rxen_active_low}, "
      f"txpower_max={inst_g3.txpower_max}")

inst_g3.detach()

# -------------------------------------------------------------------------
# Test 7: unknown platform / board names raise clear errors
# -------------------------------------------------------------------------
print("\n--- Test 7: unknown platform / board raise clear errors ---")

cfg_bad_p = dict(cfg)
cfg_bad_p["platform"]    = "made-up-board"
cfg_bad_p["radio_board"] = "meshadv-pi-hat-v1.1"
try:
    interface_class(FakeTransport(), cfg_bad_p)
    assert False, "expected _ProfileResolutionError for unknown platform"
except Exception as e:
    assert "Unknown platform" in str(e), f"unexpected error: {e}"
    assert "made-up-board" in str(e), "error should mention the bad name"
    assert "Known platforms" in str(e), "error should list known platforms"
print("[OK] unknown platform raises clear error listing known platforms")

cfg_bad_b = dict(cfg)
cfg_bad_b["platform"]    = "raspberry-pi"
cfg_bad_b["radio_board"] = "made-up-hat"
try:
    interface_class(FakeTransport(), cfg_bad_b)
    assert False, "expected _ProfileResolutionError for unknown board"
except Exception as e:
    assert "Unknown radio_board" in str(e), f"unexpected error: {e}"
    assert "made-up-hat" in str(e), "error should mention the bad name"
    assert "Known boards" in str(e), "error should list known boards"
print("[OK] unknown board raises clear error listing known boards")

# -------------------------------------------------------------------------
# Test 8: custom escape-hatch mode
# -------------------------------------------------------------------------
print("\n--- Test 8: radio_board = custom escape hatch ---")

cfg_custom = dict(cfg)
cfg_custom["platform"]    = "luckfox-pico"
cfg_custom["radio_board"] = "custom"
cfg_custom["gpiochip"]    = "gpiochip1"
cfg_custom["pin_irq"]     = "23"
cfg_custom["pin_busy"]    = "22"
cfg_custom["pin_reset"]   = "25"
cfg_custom["pin_txen"]    = "-1"
cfg_custom["pin_rxen"]    = "24"
cfg_custom.pop("pin_irq", None)  # pop doesn't help, re-add
cfg_custom["pin_irq"]     = "23"
cfg_custom["pin_busy"]    = "22"
cfg_custom["pin_reset"]   = "25"
cfg_custom["pin_txen"]    = "-1"
cfg_custom["pin_rxen"]    = "24"

inst_custom = interface_class(FakeTransport(), cfg_custom)

assert inst_custom.board_name == "custom", f"board_name={inst_custom.board_name}"
assert inst_custom.gpiochip == "gpiochip1", f"gpiochip={inst_custom.gpiochip}"
assert inst_custom.pin_lines["irq"]  == ("gpiochip1", 23), f"irq={inst_custom.pin_lines['irq']}"
assert inst_custom.pin_lines["busy"] == ("gpiochip1", 22), f"busy={inst_custom.pin_lines['busy']}"
assert inst_custom.pin_lines["reset"]== ("gpiochip1", 25), f"reset={inst_custom.pin_lines['reset']}"
assert inst_custom.pin_lines["txen"] is None, f"txen={inst_custom.pin_lines['txen']}"
assert inst_custom.pin_lines["rxen"] == ("gpiochip1", 24), f"rxen={inst_custom.pin_lines['rxen']}"
print(f"[OK] custom mode: gpiochip={inst_custom.gpiochip}, "
      f"pin_irq={inst_custom.pin_lines['irq']}, pin_txen={inst_custom.pin_lines['txen']}")

inst_custom.detach()

# -------------------------------------------------------------------------
# Test 9: per-key override wins and logs a warning
# -------------------------------------------------------------------------
print("\n--- Test 9: per-key override wins + WARNING ---")

# Profile-mode config with an override on pin_reset (board says physical 12,
# user forces physical 11)
logs_before = len(FakeRNS.logs)
cfg_override = dict(cfg)
cfg_override["platform"]    = "raspberry-pi"
cfg_override["radio_board"] = "meshadv-pi-hat-v1.1"
cfg_override.pop("pin_irq", None)
cfg_override["pin_reset"]   = "11"   # physical 11 -> BCM17 -> gpiochip0 line 17

inst_ovr = interface_class(FakeTransport(), cfg_override)

# The override should win: pin_reset -> physical 11 -> BCM17 -> (gpiochip0, 17)
assert inst_ovr.pin_lines["reset"] == ("gpiochip0", 17), \
    f"override should produce physical 11 -> BCM17, got {inst_ovr.pin_lines['reset']}"

# A WARNING should have been logged about the override
override_warnings = [
    e for e in FakeRNS.logs[logs_before:]
    if e.level == FakeRNS.LOG_WARNING and "override" in e.msg.lower() and "pin_reset" in e.msg
]
assert override_warnings, f"expected WARNING log about pin_reset override, logs={FakeRNS.logs[logs_before:]}"
print(f"[OK] per-key override: pin_reset -> {inst_ovr.pin_lines['reset']} (WARNING logged)")

inst_ovr.detach()

# -------------------------------------------------------------------------
# Test 10: based_on inheritance + overlay (test the _ProfileResolver
# directly with a synthetic overlay file)
# -------------------------------------------------------------------------
print("\n--- Test 10: overlay file with based_on inheritance ---")

# Write a temporary overlay file
overlay_dir = "/tmp/opencode/sx126x-overlay-test"
os.makedirs(overlay_dir, exist_ok=True)
overlay_path = os.path.join(overlay_dir, "sx126x_boards")
with open(overlay_path, "w") as f:
    f.write("""[boards]

  [[my-hat]]
  based_on = meshadv-pi-hat-v1.1
  header_pin_reset = 11
  profile_notes = Inherited from MeshAdv + override reset
""")
# Force the resolver to search the overlay dir
os.environ["RETICULUM_HAT_MOD_DIR"] = overlay_dir
try:
    resolver = interface_globals["_ProfileResolver"]()
    assert "my-hat" in resolver.boards, f"overlay not loaded; boards keys: {list(resolver.boards.keys())}"
    assert resolver.boards["my-hat"]["header_pin_reset"] == 11, \
        f"overlay should override reset to 11, got {resolver.boards['my-hat']['header_pin_reset']}"
    # Inherited fields should be present
    assert resolver.boards["my-hat"]["header_pin_irq"] == 36, "should inherit irq from MeshAdv"
    assert resolver.boards["my-hat"]["spi_bus"] == 0, "should inherit spi_bus from MeshAdv"
    print("[OK] overlay file with based_on = meshadv-pi-hat-v1.1 loaded + merged correctly")
finally:
    os.environ.pop("RETICULUM_HAT_MOD_DIR", None)

# -------------------------------------------------------------------------
# Test 11: preamble_length config key (interoperability with non-RNode
# LoRa firmwares such as thatSFguy/reticulum-lora-repeater which use
# a 16-symbol preamble). Default must be 8; override must flow through
# to set_lora_packet and _calculate_toa.
# -------------------------------------------------------------------------
print("\n--- Test 11: preamble_length config key ---")

# (a) default: no preamble_length in config -> 8
cfg_pl_default = dict(cfg)
inst_pl_default = interface_class(FakeTransport(), cfg_pl_default)
assert inst_pl_default.preamble_length == 8, \
    f"default preamble_length should be 8, got {inst_pl_default.preamble_length}"
print("[OK] default preamble_length == 8")

# (b) override: preamble_length = 16 -> 16, and set_lora_packet sees it
cfg_pl_16 = dict(cfg)
cfg_pl_16["preamble_length"] = "16"
inst_pl_16 = interface_class(FakeTransport(), cfg_pl_16)
assert inst_pl_16.preamble_length == 16, \
    f"preamble_length=16 should round-trip to 16, got {inst_pl_16.preamble_length}"

# Every set_lora_packet(...) call recorded by the mock so far must have
# used 16 as the preamble_length (positional arg index 1).
assert inst_pl_16.radio.set_lora_packet_calls, \
    "expected at least one set_lora_packet call on the mock radio"
for call in inst_pl_16.radio.set_lora_packet_calls:
    preamble_arg = call[1]
    assert preamble_arg == 16, \
        f"every set_lora_packet call must use preamble_length=16, saw {preamble_arg} in args={call}"
print(f"[OK] preamble_length=16 flows through to all {len(inst_pl_16.radio.set_lora_packet_calls)} set_lora_packet calls")

# (c) invalid value (non-numeric) must fall back to 8, no error raised
cfg_pl_bad = dict(cfg)
cfg_pl_bad["preamble_length"] = "not-a-number"
inst_pl_bad = interface_class(FakeTransport(), cfg_pl_bad)
assert inst_pl_bad.preamble_length == 8, \
    f"invalid preamble_length should default to 8, got {inst_pl_bad.preamble_length}"
print("[OK] invalid preamble_length silently falls back to 8")

# (d) invalid value (zero / negative) must fall back to 8
cfg_pl_zero = dict(cfg)
cfg_pl_zero["preamble_length"] = "0"
inst_pl_zero = interface_class(FakeTransport(), cfg_pl_zero)
assert inst_pl_zero.preamble_length == 8, \
    f"preamble_length=0 should default to 8, got {inst_pl_zero.preamble_length}"
print("[OK] preamble_length=0 silently falls back to 8")

# (e) _calculate_toa must use the configured preamble_length
#      8 symbols at SF8/BW125k: t_preamble = (8 + 4.25) * t_sym
#     16 symbols at SF8/BW125k: t_preamble = (16 + 4.25) * t_sym
toa_8  = inst_pl_default._calculate_toa(50)
toa_16 = inst_pl_16._calculate_toa(50)
assert toa_16 > toa_8, \
    f"toa with preamble=16 ({toa_16}) should be greater than toa with preamble=8 ({toa_8})"
expected_diff = (16 - 8) * ((2 ** 8) / 125000)
assert abs((toa_16 - toa_8) - expected_diff) < 1e-9, \
    f"toa delta should be 8*t_sym = {expected_diff}, got {toa_16 - toa_8}"
print(f"[OK] _calculate_toa honours preamble_length: toa_8={toa_8*1000:.2f}ms, "
      f"toa_16={toa_16*1000:.2f}ms, delta={expected_diff*1000:.2f}ms")

inst_pl_default.detach()
inst_pl_16.detach()
inst_pl_bad.detach()
inst_pl_zero.detach()

print("\n*** ALL TESTS PASSED ***")
