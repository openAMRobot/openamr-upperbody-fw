# CTL-017 firmware v1: design note

Status: proposal for the CTL-017 owner (Evans) to adopt, adapt or reject. Host-tested and cross-compiled; never run on hardware. Not a safety function: the hardwired PNOZ chain removes lift motor power on E-stop independently of this firmware, CAN3 carries no safety function (D6, D13), and the lift interlock (base stopped, arms stowed or carry, P-03 item 15) is enforced by the base controller, not here.

## 1. Layers (I8 architecture rule)

| Layer | Path | Depends on |
|---|---|---|
| Core: lift state machine, checks, aux I/O | `firmware/ctl017/core` | C11 only |
| CANopen slave (CiA 301 subset) | `firmware/ctl017/canopen` | C11 only |
| Application: object dictionary, PDO binding, EMCY | `firmware/ctl017/app` | core, canopen |
| Board port NUCLEO-G474RE | `boards/nucleo_g474re` | app, ST HAL |
| Host plant model | `firmware/ctl017/sim` | core (tests and simulator only) |
| SocketCAN simulator | `tools/ctl017_sim` | app, plant, Linux |

The core never calls a HAL, a register or the CAN stack. The board port samples the inputs into `lift_inputs_t` and `aux_inputs_t` every millisecond, calls `ctl017_app_step()`, and applies the H-bridge and output commands. Swapping the board or the motor driver changes only `boards/`.

## 2. Lift state machine

States are the CAN3 message-set states. The work-package names map as: MOVING_UP / MOVING_DOWN = MOVING with the velocity sign, AT_STOP = IDLE with at_stop.

| From | Event | To |
|---|---|---|
| INIT | inputs settled (0x2101 init time), exactly one stop sensor active, Hall supply good | IDLE, position referenced to that stop (P-03 item 24) |
| INIT | no sensor active | UNHOMED |
| INIT | two sensors (F09) or both end switches (F01) | FAULT |
| IDLE | MOVE_TO_STOP accepted | MOVING |
| UNHOMED or IDLE | START_HOMING accepted | HOMING (or stays and references at once if a sensor is active) |
| MOVING, HOMING | target sensor active and Hall position at the stop minus the coast lead | STOPPING |
| MOVING, HOMING | STOP, NMT leaves OPERATIONAL, or motor power lost | STOPPING (target discarded, no fault) |
| MOVING, HOMING, STOPPING | any fault | FAULT (H-bridge off at once) |
| STOPPING | standstill confirmed | IDLE if homed, position valid, no fault; UNHOMED otherwise |
| FAULT | CLEAR_FAULT with all conditions gone | IDLE or UNHOMED by the same rule; never moves |

Readiness and command rules follow S2 sections 5.3 and 6 with the D1 to D20 changes: latched fault bitmask, CLEAR_FAULT clears every clearable fault, recovery_required = not homed, no automatic resume after any stop, fault or power loss.

## 3. Checks while driving

| Check | Rule | Fault |
|---|---|---|
| Command supervision | no keep-alive (same RPDO1 again) or no core heartbeat for 300 ms (D12 alternative a, D15) | F07 |
| Travel timeout | per move: distance / expected speed x factor + margin; homing uses the full stop-0-to-stop-2 travel plus margins (P-03 item 30 A7) | F04 |
| Hall window | after a start grace, the count rate over each window must sit inside the min and max for the move or homing duty (P-03 item 30 A7; Alex 24 Oct: implausible rate gives F02) | F02 |
| Stall | no Hall pulse for the stall time while driving (Alex 24 Oct) | F03 |
| Direction | count running against the commanded direction by more than the allowance | F02 |
| Current | above the limit for the trip time, after the inrush blanking | F03 |
| Stop-sensor cross-check | at each stop sensor switching on: Hall position within the cross-check window of the expected edge position, then the count is re-referenced (P-03 item 24: "each stop cross-checks the Hall count") | F02 |
| Missing target sensor | Hall position passes the target by more than the overrun limit without its sensor | F02 |
| End switch | end switch reached in the direction of travel (stops sit 5 mm inside the switches) | F02 |
| Two stop sensors | two or more active at any time (P-03 item 24) | F09 (proposal) |
| Standstill | not confirmed within the standstill timeout after the drive is switched off (CAN3 v1 decision, platform-fw #12; start 500 ms) | F08 |
| Hall supply | lost while moving: F05; at standstill: position invalid, no fault (Alex 24 Oct) | F05 |
| Motor power | lost while moving: stop, keep counting, no fault (D4, D13); a new move is refused with cmd_result 11 | none |

Stop-sensor priority: a move ends only when the target stop's sensor is active and the Hall position has reached the stop; the sensor confirms the stop, the Hall count places the lift inside the sensor band and cross-checks the sensor. An open sensor wire reads as "no target", so the overrun limit and the travel timeout stop the lift.

Homing: START_HOMING drives down at the homing duty. The first stop sensor reached sets the reference (P-03 item 24: "between stops the node moves slowly to the nearest sensor"). With `homing_continue_to_stop0` (default, CTL-017 v1 task: "homing to L1000") homing continues to L1000 and cross-checks each sensor on the way (D10: home down). A sensor active at power-up gives a valid position without homing.

## 4. CANopen: own implementation instead of CANopenNode

D1 names CANopenNode. This proposal implements its own small CiA 301 slave (`firmware/ctl017/canopen`, about 800 lines with its header) because:
- CANopenNode is Apache-2.0. Vendoring it needs its licence file and notices in this tree, and this change may not add licence files; a fetched (not vendored) copy would also work but adds a build dependency to a node that needs only NMT, heartbeat, expedited SDO, two TPDOs, two RPDOs and EMCY.
- The small slave runs unchanged in the host tests and the SocketCAN simulator, so every PDO byte is tested.

The object dictionary uses standard CiA 301 indices and a generated EDS, so moving to CANopenNode later changes the app and canopen layers only. Not supported: segmented and block SDO, SYNC, TIME, LSS, dynamic PDO mapping, parameter storage (0x1010/0x1011). This is a decision for the owner (see the report).

## 5. Parameters

All tunables are in `lift_config_t` (`firmware/ctl017/core/include/ctl017/lift_config.h`) with the source or TODO(Evans) of each value, and writable in PRE-OPERATIONAL through 0x2100 and 0x2101 for bench calibration (volatile in v1). Decided values: stop tolerance plus or minus 2 mm (D3), 300 ms command deadline and heartbeat consumer, 100 ms heartbeat and TPDO1 event time (D15), F08 start value 500 ms, halt = H-bridge disabled (I8 rev 3 Q10), node 0x10, core 0x01, 500 kbit/s. Everything else is a placeholder.

## 6. Tests

```sh
cmake -S . -B build && cmake --build build
sudo tools/ctl017_sim/setup_vcan.sh vcan0     # once per boot, for the vcan test
ctest --test-dir build --output-on-failure
```

| ctest | What |
|---|---|
| test_lift | 38 core tests against the plant model, with ASan and UBSan |
| test_canopen | 16 bus-level tests: NMT, heartbeat, SDO, PDO bytes, EMCY, parameters |
| od_docs_up_to_date | `docs/ctl017.eds` and the table in `docs/ctl017_object_dictionary.md` match the firmware OD |
| vcan_pytest | 6 python-can tests against `ctl017_sim` on vcan0 (skips without vcan) |

Verification matrix items covered by host tests (CAN3 message set section 14, Alex: "T05 to T08, T13 to T19, T24 and T26 to T34 run as host tests"): T03, T06, T13, T14, T15, T16, T17, T18, T24, T26, T27, T28 (v1 form: no position-class exception), T29, T30, T31, T32, T33, T34. T04, T05, T07, T08 and T21 are base-controller or host rules; the reference decoder in `tests/ctl017_vcan/ctl017_decode.py` implements the counter rule. T01, T02, T09 to T12, T20, T22, T23 and T25 need the bench or the other repositories.
