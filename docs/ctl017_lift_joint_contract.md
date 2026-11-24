# lift_joint from the CTL-017 PDOs: contract note for the host side

Status: proposal for the owners of the base controller relay (openamr-platform-fw), the host adapter (openamr-platform-sw, interfaces #31 Q5) and the robot model (openamr-upperbody-sw). It changes no other repository. It follows the I8 base controller interface rev 3 section 5.3 (frozen 6 Nov 2026) and the CAN3 message set in `docs/ctl017_object_dictionary.md`.

## 1. Chain

```
CTL-017 (node 0x10)  --CAN3 TPDO1/TPDO2-->  base controller  --/base/aux_status 5 Hz-->  host adapter  --/joint_states-->  lift_joint
```

- CTL-017 never publishes ROS topics. The base controller relays the aux-device block on /base/aux_status (5 Hz heartbeat, stale after 0.6 s, reliable, transient-local depth 1).
- A host adapter in platform-sw republishes `lift_joint` into /joint_states from that block only while the position is valid (interfaces #31 Q5). Firmware does not publish /joint_states.

## 2. Position

| Step | Rule |
|---|---|
| CAN field | TPDO1 bytes 2-3, `position`, u16, 0.1 mm above the lower end of travel, 0xFFFF when unknown |
| Zero of lift_joint | stop 0 (L1000). Its position above the lower end is object 0x2100:01 (INTEGER32, micrometres; default 5000, calibrated by the owner) |
| Base controller | reads 0x2100:01 by SDO after every CTL-017 boot-up message, before NMT start, and keeps it with the node session |
| Formula | `lift_joint_m = position * 1e-4 - stop0_um * 1e-6` |
| Range | 0 to 0.350 m (I8 rev 3). L1000 = 0.000, L1175 = 0.175, L1350 = 0.350 with the default calibration |
| Out of range | within 2 mm (D3 tolerance) outside 0 to 0.350 m: clamp to the joint limit; further outside: treat as invalid and raise a diagnostic (calibration or stroke error) |

Shoulder height = 1.000 m + lift_joint_m at the OpenArm arm mount point (lift baseline in the README).

## 3. Velocity

| Step | Rule |
|---|---|
| CAN field | TPDO2 bytes 4-5, `velocity`, i16, 0.1 mm/s, positive up, measured from the Hall count over 100 ms, 0 at standstill |
| Formula | `lift_joint_velocity_m_s = velocity * 1e-4` |
| Rate | TPDO2 goes out at least every 100 ms (0x1801:05) |

Effort is not provided. Motor current (TPDO2 bytes 2-3, mA) is a drive diagnostic, not a joint force; leave `effort` empty.

## 4. Validity and staleness

`lift_joint` is published only while every condition holds; otherwise nothing is published for it (no stale value is repeated) and the head-camera transform is invalid.

| Layer | Condition | Owner |
|---|---|---|
| CTL-017 | TPDO1 flag bit 0 `position_valid` = 1 and position is not 0xFFFF | CTL-017 |
| CAN3 | CTL-017 heartbeat seen within 300 ms; TPDO1 received within 300 ms; status_counter progressing (gap limit 3, resynchronised after 3 frames, D15) | base controller |
| ROS | /base/aux_status received within 0.6 s (local receipt time; the header stamp is diagnostic only) | host adapter |
| Restore | after any loss, valid again after three fresh, progressing TPDO1 frames with position_valid = 1 (D8) | base controller and host adapter |

A node restart (boot-up message 0x710 with data 0x00, or status_counter restarting with cmd_result 0xFF) invalidates the position on the host until the rules above hold again; after a CTL-017 power loss the node needs homing unless a stop sensor is active at power-up.

Stamp: the host adapter stamps the JointState with the receipt time of the /base/aux_status message it used.

## 5. Other aux-block lift fields (I8 rev 3 section 5.3)

| /base/aux_status field | From |
|---|---|
| position_valid | TPDO1 flag bit 0, and the CAN3 conditions of section 4 |
| moving and direction | TPDO1 flag bit 2; direction from the sign of TPDO2 velocity |
| motor current | TPDO2 bytes 2-3 |
| target stop | TPDO2 byte 1 (0 L1000, 1 L1175, 2 L1350, 0xFF none) |
| at_stop | TPDO1 byte 4 `active_stop_id` (0, 1, 2 = L1000, L1175, L1350; 0xFF = none); flag bit 1 is set exactly when it is not 0xFF |
| three stop-sensor states | TPDO2 byte 0 bits 0 to 2 |
| fault | TPDO1 flag bit 5; fault ID from EMCY (0xFF01 to 0xFF09), 0x1003 or TPDO2 bytes 6 and 7 bit 0 (F09) |
| second E-stop | TPDO2 byte 0 bit 3: PRESSED when 1, RELEASED when 0, UNKNOWN whenever the CTL-017 link is lost (section 4) |
| head light on, Getac power | TPDO2 byte 7 bits 1 and 2 (commanded state; no light current feedback in v1) |

"Robot ready" needs the lift at a calibrated stop (interfaces #31 Q13): `active_stop_id` is not 0xFF and the position is valid.

## 6. Reference implementation

`tests/ctl017_vcan/ctl017_decode.py` decodes TPDO1 and TPDO2, applies the position and velocity formulas and the status_counter rule. The vcan test checks `lift_joint` = 0.175 m within 2 mm at L1175 against the firmware running in `tools/ctl017_sim`.
