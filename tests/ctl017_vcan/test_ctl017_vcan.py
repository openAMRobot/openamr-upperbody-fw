# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""CTL-017 over virtual CAN: the firmware application and the lift plant run
in tools/ctl017_sim; this test plays the base controller (NMT master,
heartbeat 0x701, LiftCommand keep-alive) with python-can.

Needs a vcan interface (tools/ctl017_sim/setup_vcan.sh) and the simulator
binary in CTL017_SIM. Skips otherwise."""
import os
import struct
import subprocess
import threading
import time

import pytest

can = pytest.importorskip("can")

from ctl017_decode import (EMCY_ID, FLAG_AT_STOP, FLAG_MOTOR_POWER, FLAG_POSITION_VALID, HB_ID,
                           RPDO1_ID, SDO_RX, SDO_TX, TPDO1_ID, TPDO2_ID, CounterMonitor,
                           LiftStatus, LiftStatus2, lift_joint)

IFACE = os.environ.get("CTL017_VCAN", "vcan0")
SIM = os.environ.get("CTL017_SIM")
SIM_CTRL = 0x7F0


def _iface_up():
    return os.path.exists("/sys/class/net/%s" % IFACE)


pytestmark = pytest.mark.skipif(not (SIM and os.path.exists(SIM) and _iface_up()),
                                reason="needs CTL017_SIM and a vcan interface")


class Master:
    """Minimal base-controller side: heartbeat, frame log, keep-alive."""

    def __init__(self, bus):
        self.bus = bus
        self.frames = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.heartbeat = True
        self.keepalive = None  # (seq, op, stop)
        self.rx = threading.Thread(target=self._rx, daemon=True)
        self.tx = threading.Thread(target=self._tx, daemon=True)
        self.rx.start()
        self.tx.start()

    def _rx(self):
        while not self.stop.is_set():
            m = self.bus.recv(0.05)
            if m is not None and not m.is_extended_id:
                with self.lock:
                    self.frames.append((time.monotonic(), m.arbitration_id, bytes(m.data)))

    def _tx(self):
        while not self.stop.is_set():
            if self.heartbeat:
                self.send(0x701, [0x05])
            ka = self.keepalive
            if ka is not None:
                self.send(RPDO1_ID, list(ka))
            time.sleep(0.1)

    def send(self, cid, data):
        self.bus.send(can.Message(arbitration_id=cid, data=bytes(data), is_extended_id=False))

    def since(self, t0, cid):
        with self.lock:
            return [(t, d) for (t, i, d) in self.frames if i == cid and t >= t0]

    def wait_for(self, cid, pred, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            for _, d in self.since(t0 - 0.0, cid):
                if pred(d):
                    return d
            time.sleep(0.01)
        return None

    def latest(self, cid):
        with self.lock:
            for (_, i, d) in reversed(self.frames):
                if i == cid:
                    return d
        return None

    def sdo_read(self, index, sub, timeout=1.0):
        t0 = time.monotonic()
        self.send(SDO_RX, [0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])
        while time.monotonic() - t0 < timeout:
            for _, d in self.since(t0, SDO_TX):
                if d[1] | (d[2] << 8) == index and d[3] == sub:
                    return d
            time.sleep(0.005)
        return None

    def command(self, seq, op, stop):
        self.keepalive = (seq, op, stop)
        self.send(RPDO1_ID, [seq, op, stop])

    def close(self):
        self.stop.set()
        self.rx.join(1)
        self.tx.join(1)


@pytest.fixture
def node(request):
    start_mm = getattr(request, "param", 180)
    bus = can.Bus(interface="socketcan", channel=IFACE, receive_own_messages=False)
    master = Master(bus)
    t_start = time.monotonic()
    proc = subprocess.Popen([SIM, "--iface", IFACE, "--start-mm", str(start_mm)],
                            stderr=subprocess.PIPE)
    try:
        boot = master.wait_for(HB_ID, lambda d: d == b"\x00", 2.0)
        assert boot is not None, "no boot-up message"
        time.sleep(0.2)
        yield master, t_start
    finally:
        master.keepalive = None
        proc.terminate()
        proc.wait(2)
        master.close()
        bus.shutdown()


def test_boot_heartbeat_and_identity(node):
    m, _ = node
    t0 = time.monotonic()
    time.sleep(1.05)
    hbs = m.since(t0, HB_ID)
    assert 9 <= len(hbs) <= 12
    assert all(d == b"\x7f" for _, d in hbs)  # PRE-OPERATIONAL
    d = m.sdo_read(0x1018, 2)
    assert d is not None and d[0] == 0x43
    assert struct.unpack("<I", d[4:8])[0] == 0x11
    d = m.sdo_read(0x1017, 0)
    assert struct.unpack("<H", d[4:6])[0] == 100
    assert m.since(t0, TPDO1_ID) == []


def test_status_at_l1175_and_lift_joint(node):
    m, _ = node
    m.send(0x000, [0x01, 0x10])
    d = m.wait_for(TPDO1_ID, lambda d: True, 1.0)
    assert d is not None
    st = LiftStatus.decode(d)
    assert st.state == 3  # IDLE
    assert st.flags & FLAG_POSITION_VALID and st.flags & FLAG_AT_STOP and st.flags & FLAG_MOTOR_POWER
    assert st.active_stop_id == 1
    assert st.cmd_result == 0xFF
    stop0 = struct.unpack("<i", m.sdo_read(0x2100, 1)[4:8])[0]
    st2 = LiftStatus2.decode(m.wait_for(TPDO2_ID, lambda d: True, 1.0))
    pos, vel = lift_joint(st, st2, stop0)
    assert abs(pos - 0.175) <= 0.002
    assert vel == 0.0
    # TPDO1 at 100 ms with a progressing counter (D15 resync after 3).
    time.sleep(0.6)
    mon = CounterMonitor()
    ok = [mon.feed(LiftStatus.decode(d).status_counter) for _, d in m.since(time.monotonic() - 0.6, TPDO1_ID)]
    assert ok[-1] is True and mon.lost_frames == 0


def test_move_l1175_to_l1350(node):
    m, _ = node
    m.send(0x000, [0x01, 0x10])
    time.sleep(0.2)
    m.command(1, 1, 2)
    d = m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).last_seq == 1, 1.0)
    assert d is not None and LiftStatus.decode(d).cmd_result == 0
    moving = m.wait_for(TPDO2_ID, lambda d: LiftStatus2.decode(d).velocity_01mm_s > 100, 2.0)
    assert moving is not None and LiftStatus2.decode(moving).target_stop_id == 2
    done = m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).active_stop_id == 2, 20.0)
    m.keepalive = None
    assert done is not None, "did not reach L1350"
    st = LiftStatus.decode(done)
    assert st.state == 3 and abs(st.position_01mm - 3550) <= 20


def test_keepalive_loss_gives_emcy_f07(node):
    m, _ = node
    m.send(0x000, [0x01, 0x10])
    time.sleep(0.2)
    m.command(1, 1, 2)
    time.sleep(1.0)
    t0 = time.monotonic()
    m.keepalive = None
    e = m.wait_for(EMCY_ID, lambda d: d[0] | (d[1] << 8) == 0xFF07, 1.0)
    assert e is not None
    assert time.monotonic() - t0 < 0.6
    assert e[3] == 7
    st = LiftStatus.decode(m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).state == 6, 1.0))
    assert st.flags & (1 << 5)
    # CLEAR_FAULT is refused (busy) until standstill is confirmed: wait for
    # the moving flag to drop, then clear. EMCY error reset follows.
    assert m.wait_for(TPDO1_ID, lambda d: not LiftStatus.decode(d).flags & (1 << 2), 1.0) is not None
    m.command(2, 2, 0)
    m.keepalive = None
    r = m.wait_for(EMCY_ID, lambda d: d[0:2] == b"\x00\x00", 1.0)
    assert r is not None
    d = m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).last_seq == 2, 1.0)
    assert LiftStatus.decode(d).cmd_result == 0 and LiftStatus.decode(d).state == 3


def test_motor_power_absent_rejected(node):
    m, _ = node
    m.send(0x000, [0x01, 0x10])
    m.send(SIM_CTRL, [1, 0])  # motor power off (E-stop chain open)
    time.sleep(0.2)
    m.command(5, 1, 0)
    m.keepalive = None
    d = m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).last_seq == 5, 1.0)
    assert d is not None and LiftStatus.decode(d).cmd_result == 11


def test_second_estop_reported(node):
    m, _ = node
    m.send(0x000, [0x01, 0x10])
    m.send(SIM_CTRL, [6, 1])
    d = m.wait_for(TPDO2_ID, lambda d: LiftStatus2.decode(d).inputs & 0x08, 1.0)
    assert d is not None


def test_move_refused_without_core_heartbeat(node):
    """D12 alternative (a): no core heartbeat 0x701, MOVE_TO_STOP refused with
    cmd_result 12 and no motion; accepted once the heartbeat is back."""
    m, _ = node
    m.heartbeat = False
    time.sleep(0.5)  # consumer times out (300 ms)
    m.send(0x000, [0x01, 0x10])
    time.sleep(0.1)
    m.command(7, 1, 2)
    d = m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).last_seq == 7, 1.0)
    assert d is not None and LiftStatus.decode(d).cmd_result == 12
    time.sleep(0.4)
    st = LiftStatus.decode(m.latest(TPDO1_ID))
    assert st.state == 3 and not st.flags & 0x04  # IDLE, not moving
    m.keepalive = None
    m.heartbeat = True
    time.sleep(0.3)
    m.command(8, 1, 2)
    d = m.wait_for(TPDO1_ID, lambda d: LiftStatus.decode(d).last_seq == 8, 1.0)
    assert d is not None and LiftStatus.decode(d).cmd_result == 0
    m.command(9, 0, 0)
