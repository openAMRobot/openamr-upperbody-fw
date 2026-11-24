# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""Reference decoder of the CTL-017 PDOs and the lift_joint derivation of
docs/ctl017_lift_joint_contract.md. Used by the vcan tests; the base
controller and the host adapter implement the same rules."""
import struct
from dataclasses import dataclass
from typing import Optional

NODE_ID = 0x10
TPDO1_ID = 0x180 + NODE_ID
TPDO2_ID = 0x280 + NODE_ID
RPDO1_ID = 0x200 + NODE_ID
RPDO2_ID = 0x300 + NODE_ID
EMCY_ID = 0x080 + NODE_ID
HB_ID = 0x700 + NODE_ID
SDO_RX = 0x600 + NODE_ID
SDO_TX = 0x580 + NODE_ID

FLAG_POSITION_VALID = 1 << 0
FLAG_AT_STOP = 1 << 1
FLAG_MOVING = 1 << 2
FLAG_FAULT_PRESENT = 1 << 5
FLAG_MOTOR_POWER = 1 << 6
FLAG_RECOVERY_REQUIRED = 1 << 7

STATES = ["INIT", "UNHOMED", "HOMING", "IDLE", "MOVING", "STOPPING", "FAULT"]
STALE_AFTER_S = 0.6        # /base/aux_status stale time (I8 rev 3)
POSITION_UNKNOWN = 0xFFFF


@dataclass
class LiftStatus:
    state: int
    flags: int
    position_01mm: int
    active_stop_id: int
    last_seq: int
    cmd_result: int
    status_counter: int

    @classmethod
    def decode(cls, data: bytes) -> "LiftStatus":
        if len(data) != 8:
            raise ValueError("TPDO1 must be 8 bytes")
        s, f, pos, stop, seq, res, cnt = struct.unpack("<BBHBBBB", data)
        return cls(s, f, pos, stop, seq, res, cnt)

    @property
    def position_valid(self) -> bool:
        return bool(self.flags & FLAG_POSITION_VALID) and self.position_01mm != POSITION_UNKNOWN


@dataclass
class LiftStatus2:
    inputs: int
    target_stop_id: int
    motor_current_ma: int
    velocity_01mm_s: int
    faults_lo: int
    aux_flags: int

    @classmethod
    def decode(cls, data: bytes) -> "LiftStatus2":
        if len(data) != 8:
            raise ValueError("TPDO2 must be 8 bytes")
        return cls(*struct.unpack("<BBHhBB", data))

    @property
    def faults(self) -> int:
        return self.faults_lo | ((self.aux_flags & 0x01) << 8)


def lift_joint(status: LiftStatus, status2: Optional[LiftStatus2], stop0_position_um: int):
    """Returns (position_m, velocity_m_s) of lift_joint, or None when the
    position must not be published (position_valid false)."""
    if not status.position_valid:
        return None
    position_m = (status.position_01mm * 100 - stop0_position_um) / 1e6
    velocity_m_s = (status2.velocity_01mm_s / 1e4) if status2 is not None else 0.0
    return position_m, velocity_m_s


class CounterMonitor:
    """status_counter progression (D15): gap limit 3, resync after 3 frames."""

    def __init__(self, gap_limit=3, resync=3):
        self.gap_limit = gap_limit
        self.resync = resync
        self.last = None
        self.good = 0
        self.lost_frames = 0

    def feed(self, counter: int) -> bool:
        """Returns True while progression is established."""
        if self.last is None:
            self.last = counter
            self.good = 0
            return False
        delta = (counter - self.last) % 256
        self.last = counter
        if delta == 0 or delta > self.gap_limit:
            self.good = 0  # frozen or discontinuity
            return False
        if delta > 1:
            self.lost_frames += delta - 1
        self.good += 1
        return self.good >= self.resync
