# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""CTL-017 CANopen application layer over virtual CAN.

tools/ctl017_sim runs ctl017_app with a fixed status (no lift, nothing
moves); this test plays the base controller with python-can and checks the
wire behaviour: boot-up, heartbeat, SDO, PDO, malformed frames, heartbeat
consumer EMCY.

Needs a vcan interface (tools/ctl017_sim/setup_vcan.sh), python-can and the
binary in CTL017_SIM. run_vcan_tests.py turns "not available" into ctest
SKIPPED. The config values passed to the harness are test values.
"""
import os
import struct
import subprocess
import threading
import time

import pytest

can = pytest.importorskip("can")

from ctl017_decode import EMCY_ID, HB_ID, RPDO1_ID, SDO_RX, SDO_TX, TPDO1_ID, LiftStatus

IFACE = os.environ.get("CTL017_VCAN", "vcan0")
SIM = os.environ.get("CTL017_SIM")
BASE = 0x01
TEST_ARGS = ["--base-node", str(BASE), "--hb-ms", "100", "--hbc-ms", "300", "--inhibit", "100",
             "--event-ms", "100", "--rpdo1-ms", "300", "--product", "0x11", "--revision", "0x10000"]


class Master:
    def __init__(self, bus):
        self.bus = bus
        self.frames = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.heartbeat = True
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
                self.send(0x700 + BASE, [0x05])
            time.sleep(0.1)

    def send(self, cid, data, ext=False, rtr=False):
        self.bus.send(can.Message(arbitration_id=cid, data=bytes(data), is_extended_id=ext,
                                  is_remote_frame=rtr, dlc=len(data) if not rtr else 3))

    def since(self, t0, cid):
        with self.lock:
            return [(t, d) for (t, i, d) in self.frames if i == cid and t >= t0]

    def wait_for(self, cid, pred, timeout, t0=None):
        t0 = time.monotonic() if t0 is None else t0
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for _, d in self.since(t0, cid):
                if pred(d):
                    return d
            time.sleep(0.01)
        return None

    def sdo_read(self, index, sub, timeout=1.0):
        t0 = time.monotonic()
        self.send(SDO_RX, [0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])
        return self.wait_for(SDO_TX, lambda d: (d[1] | (d[2] << 8)) == index and d[3] == sub,
                             timeout, t0)

    def close(self):
        self.stop.set()
        self.rx.join(1)
        self.tx.join(1)


@pytest.fixture
def node():
    bus = can.Bus(interface="socketcan", channel=IFACE, receive_own_messages=False)
    master = Master(bus)
    t0 = time.monotonic()
    proc = subprocess.Popen([SIM, "--iface", IFACE] + TEST_ARGS, stderr=subprocess.PIPE)
    try:
        assert master.wait_for(HB_ID, lambda d: d == b"\x00", 2.0, t0) is not None, "no boot-up"
        time.sleep(0.2)
        yield master
    finally:
        proc.terminate()
        proc.wait(2)
        master.close()
        bus.shutdown()


def test_boot_heartbeat_and_identity(node):
    t0 = time.monotonic()
    time.sleep(1.05)
    hbs = node.since(t0, HB_ID)
    assert 9 <= len(hbs) <= 12
    assert all(d == b"\x7f" for _, d in hbs)
    d = node.sdo_read(0x1018, 2)
    assert d is not None and d[0] == 0x43 and struct.unpack("<I", d[4:8])[0] == 0x11
    assert node.since(t0, TPDO1_ID) == []


def test_operational_status_is_unknown(node):
    node.send(0x000, [0x01, 0x10])
    d = node.wait_for(TPDO1_ID, lambda d: True, 1.0)
    assert d is not None
    st = LiftStatus.decode(d)
    assert st.state == 0 and st.position_01mm == 0xFFFF


def test_malformed_frames_ignored(node):
    node.send(0x000, [0x01, 0x10], ext=True)       # 29-bit NMT start
    node.send(0x000, [0x01, 0x10, 0x00])           # wrong length
    node.send(RPDO1_ID, [], rtr=True)
    time.sleep(0.3)
    t0 = time.monotonic()
    time.sleep(0.25)
    assert all(d == b"\x7f" for _, d in node.since(t0, HB_ID))


def test_heartbeat_loss_gives_emcy_8130(node):
    t0 = time.monotonic()
    node.heartbeat = False
    d = node.wait_for(EMCY_ID, lambda d: d[0] == 0x30 and d[1] == 0x81, 1.5, t0)
    assert d is not None and (d[2] & 0x10)
    node.heartbeat = True
    assert node.wait_for(EMCY_ID, lambda d: d[0] == 0 and d[1] == 0, 1.0, t0) is not None
