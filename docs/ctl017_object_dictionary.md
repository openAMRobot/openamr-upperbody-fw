# CTL-017 CANopen interface and object dictionary (proposal v1)

Status: proposal for the CTL-017 owner (Evans) to adopt, adapt or reject. Nothing here is hardware-tested. The firmware in `firmware/ctl017` implements exactly this document; the object table at the end and `docs/ctl017.eds` are generated from the firmware and a host test fails if they drift apart.

Sources: the CAN3 lift message set (openamr-platform-fw #12: proposal of 23 Oct 2026, decisions D1 to D20 of 24 Oct, accepted 25 Oct, and the changes of 27 Oct) and P-03 item 29 and item 30 A7 (CTL-017 board and scope, decision D-11). CAN3 carries no safety function. The hardwired PNOZ chain removes lift motor power independently of this interface.

## 1. Bus and identifiers

| Item | Value | Source |
|---|---|---|
| Bus | CAN3 (base controller FDCAN3), classic CAN, CANopen | D1, P-03 item 29 |
| Bit rate | 500 kbit/s | D1 |
| CTL-017 node-ID | 0x10 | Alex 24 Oct, P-03 item 29 |
| NMT master and heartbeat consumer target | base controller, node-ID 0x01 | Alex 24 Oct |
| Profile | none (device type 0x00000000), custom lift profile, not CiA 402 | D1 |

| Object | COB-ID | Direction |
|---|---|---|
| NMT | 0x000 | base controller to CTL-017 |
| EMCY | 0x090 | CTL-017 to base controller |
| TPDO1 LiftStatus | 0x190 | CTL-017 to base controller |
| TPDO2 LiftStatus2 | 0x290 | CTL-017 to base controller |
| RPDO1 LiftCommand | 0x210 | base controller to CTL-017 |
| RPDO2 AuxCommand | 0x310 | base controller to CTL-017 |
| SDO server | rx 0x610, tx 0x590 | |
| Heartbeat CTL-017 | 0x710 | CTL-017 to base controller |
| Heartbeat consumed | 0x701 | base controller to CTL-017 |

## 2. NMT, heartbeat and supervision

- Boot: CTL-017 sends the boot-up message (0x710, data 0x00) and enters PRE-OPERATIONAL. PDOs run only in OPERATIONAL; the base controller starts the node with NMT start (0x01). SDO is served in PRE-OPERATIONAL and OPERATIONAL, not in STOPPED.
- Leaving OPERATIONAL (NMT stop or pre-operational) halts the lift at once: no RPDO can arrive any more.
- NMT reset node (0x81) halts the lift and requests a system reset from the board port. Reset communication (0x82) re-sends the boot-up message and returns to PRE-OPERATIONAL.
- Heartbeat producer 0x1017 = 100 ms. Heartbeat consumer 0x1016:01 = node 0x01, 300 ms (D15). Monitoring starts with the first heartbeat received.
- Command supervision (D12 alternative a): while a move or homing is active, the lift stops and raises F07 if either the command keep-alive or the core heartbeat is missing for 300 ms. The keep-alive is RPDO1 re-sent every 100 ms with the same seq and identical content (S2 section 11). The deadline is 0x1400:05 (RPDO1 event timer, CiA 301 deadline monitoring); it is armed only while a move or homing is active, so an idle base controller does not need to send RPDO1.
- The base controller watches the CTL-017 heartbeat (node lost after 300 ms) and the TPDO1 status_counter (gap limit 3, resync after 3 frames, D15). Those rules run on the base controller and are listed here for completeness.

## 3. TPDO1 LiftStatus (0x190, 8 bytes)

Event driven (transmission type 0xFE): sent on any change of bytes 0 to 6, at most every inhibit time 0x1800:03 (default 10 ms, TODO(Evans)), and at least every event time 0x1800:05 (100 ms). Little-endian.

| Byte | Field | Object | Meaning |
|---|---|---|---|
| 0 | state | 0x2000:01 | 0 INIT, 1 UNHOMED, 2 HOMING, 3 IDLE, 4 MOVING, 5 STOPPING, 6 FAULT |
| 1 | flags | 0x2000:02 | see below |
| 2-3 | position | 0x2000:03 | u16, 0.1 mm above the lower end of travel; 0xFFFF when position_valid is 0 |
| 4 | active_stop_id | 0x2000:04 | stop the lift is confirmed at (0, 1, 2), 0xFF otherwise; never the target |
| 5 | last_seq | 0x2000:05 | seq of the last LiftCommand processed; 0 after start-up |
| 6 | cmd_result | 0x2000:06 | result of that command (section 6); 0xFF = no command since start-up |
| 7 | status_counter | 0x2000:07 | +1 on every TPDO1 frame, wraps 255 to 0, restarts at 0 on node start |

Flags (after Alex 27 Oct):

| Bit | Flag | Rule in the firmware |
|---|---|---|
| 0 | position_valid | Hall position referenced to a stop sensor and Hall supply good |
| 1 | at_stop | 1 if and only if active_stop_id is not 0xFF: position valid, not moving, the stop's sensor active and the Hall position within plus or minus 2 mm (D3) of the stop |
| 2 | moving | H-bridge driving, or Hall count changed within the standstill quiet time |
| 3 | lower_limit | DOLD lower end switch (only if wired, TODO(Evans)) |
| 4 | upper_limit | DOLD upper end switch (only if wired) |
| 5 | fault_present | latched fault bitmask is not 0; read 0x1003 or 0x2002 by SDO, or TPDO2 bytes 6 and 7 |
| 6 | motor_power_present | motor-supply sense (functional status, not a safety signal) |
| 7 | recovery_required | not homed (D9) |

The work package names the states IDLE, MOVING_UP, MOVING_DOWN, AT_STOP, FAULT, HOMING. On the wire they are the S2 states: MOVING_UP and MOVING_DOWN are MOVING with the sign of the TPDO2 velocity, AT_STOP is IDLE with at_stop set.

## 4. TPDO2 LiftStatus2 (0x290, 8 bytes)

P-03 item 24 and the 27 Oct decision put the stop sensor states, motor current and target stop in TPDO2. Layout proposal (TODO(Evans) to confirm). Sent on change of bytes 0, 1, 6 or 7 (inhibit 0x1801:03, default 10 ms) and every 0x1801:05 (default 100 ms); current and velocity go out with the event timer only.

| Byte | Field | Object | Meaning |
|---|---|---|---|
| 0 | inputs | 0x2001:01 | bit 0 to 2: stop sensor L1000, L1175, L1350 active (debounced); bit 3: second E-stop spare NO closed (pressed); bit 4: Hall supply good; bit 5 to 7: spare inputs 1 to 3 |
| 1 | target_stop_id | 0x2001:02 | target of the move or homing in progress, 0xFF none |
| 2-3 | motor_current | 0x2001:03 | u16, mA, shared current sense |
| 4-5 | velocity | 0x2001:04 | i16, 0.1 mm/s, positive up, from the Hall count over 100 ms; 0 at standstill |
| 6 | faults F01 to F08 | 0x2001:05 | bit n-1 = Fn |
| 7 | aux flags | 0x2001:06 | bit 0: F09; bit 1: head light on; bit 2: Getac enable on; bits 3 to 7: 0 |

The second E-stop bit is a diagnostic observation only. The base controller reports RELEASED, PRESSED or UNKNOWN in /base/aux_status and UNKNOWN whenever CTL-017 is lost (I8 rev 3 section 5.3).

## 5. RPDO1 LiftCommand (0x210, 3 bytes) and RPDO2 AuxCommand (0x310, 3 bytes)

| Byte | RPDO1 field | Object | Values |
|---|---|---|---|
| 0 | seq | 0x2010:01 | u8; a new value is a new command |
| 1 | op | 0x2010:02 | 0 STOP, 1 MOVE_TO_STOP, 2 CLEAR_FAULT, 3 START_HOMING |
| 2 | stop_id | 0x2010:03 | 0 L1000, 1 L1175, 2 L1350 (D3); ignored for other ops but part of the content compare |

- Same seq and identical content: keep-alive. Refreshes the deadline, not executed again; a late keep-alive of a completed command only echoes the earlier result.
- Same seq and different content: rejected with cmd_result 10, no refresh, no state change. The base controller then sends STOP with a new seq.
- Frames shorter than the mapping are not processed (counted in 0x2300:03).
- 0x2010 is written only by RPDO1; an SDO write is refused (abort 0x08000021), so no debug path bypasses the seq rules.

| Byte | RPDO2 field | Object | Values |
|---|---|---|---|
| 0 | head light on | 0x2011:01 | 0 or 1; on only during arm operations (P-03 item 29); off at boot |
| 1 | head light dim level | 0x2011:02 | 0 to 100 %, low-side PWM duty |
| 2 | Getac enable | 0x2011:03 | 0 or 1; boot state is a parameter, TODO(Evans) |

RPDO2 has no deadline. On loss of the core heartbeat the outputs hold by default (parameter, TODO(Evans)); the Getac enable is never dropped by a communication loss.

## 6. Commands and cmd_result

| Code | Meaning |
|---|---|
| 0 | accepted (started, not completed) |
| 1 | invalid stop_id |
| 2 | position invalid |
| 3 | fault active |
| 4 | end switch already active in the direction of travel |
| 5 | busy (start-up, move, homing or stopping in progress) |
| 6 | recovery required (not homed) |
| 7 | clearing conditions not satisfied |
| 8 | homing preconditions not satisfied (both end switches, two stop sensors or Hall supply bad) |
| 9 | unknown op |
| 10 | sequence conflict |
| 11 | motor power absent |
| 0xFF | no command since start-up |

Check order for MOVE_TO_STOP in the firmware: 1, 3, 5, 6, 2, already at that stop (accepted, no motion), 11, 4. STOP is never rejected. CLEAR_FAULT: 5 while motion is in progress; accepted no-op without a fault (D9); otherwise clears every fault whose condition is gone and answers 7 if any remain. START_HOMING: 3, 5, 8, then 11 and 4 if motion is needed.

Homing (D10, P-03 item 24, work package): if a stop sensor is active, the reference is taken there without motion; otherwise the lift drives down at the homing duty and the first stop sensor reached sets the reference. With 0x2101:30 = 1 (default) homing then continues to L1000 and cross-checks the count at each sensor on the way.

## 7. Faults, EMCY and error register

Faults are a latched bitmask (D9), cleared by CLEAR_FAULT or a node power cycle, never by themselves. The full mask is in 0x2002.

| Bit | ID | Condition in the firmware | Invalidates position |
|---|---|---|---|
| 0 | F01 | both end switches active | yes |
| 1 | F02 | Hall count inconsistent: cross-check error at a stop sensor edge above 0x2101:04, Hall rate outside the window, count against the commanded direction, end switch reached in the direction of travel, or target passed by more than 0x2101:08 without its sensor | yes |
| 2 | F03 | current above 0x2101:23 for 0x2101:24 after the inrush blanking, or no Hall pulse for 0x2101:19 while driving (stall) | no |
| 3 | F04 | travel timeout of the move (distance / speed x factor + margin) | no |
| 4 | F05 | Hall supply lost while moving | yes |
| 5 | F06 | invalid stored record; unused in v1 (no stored position, D5) | |
| 6 | F07 | command keep-alive or core heartbeat missing during a move or homing | no |
| 7 | F08 | standstill not confirmed 0x2101:28 after the drive was switched off (start 500 ms) | no |
| 8 | F09 | proposal: two or more stop sensors active (P-03 item 24 says this is a fault but assigns no ID) | yes |

"Invalidates position" clears position_valid and sets recovery_required; the lift then needs homing after the fault is cleared. Hall supply loss at standstill clears position_valid without a fault (Alex 24 Oct).

EMCY (0x090, 8 bytes), one per newly raised fault:

| Byte | Content |
|---|---|
| 0-1 | error code 0xFF00 + fault number (0xFF01 to 0xFF09); 0x0000 = error reset when the last fault clears |
| 2 | error register 0x1001 |
| 3 | fault number (1 to 9) |
| 4-5 | fault bitmask after the event |
| 6-7 | 0 |

Error register 0x1001: bit 0 generic (any fault), bit 1 current (F03), bit 4 communication (F07), bit 7 manufacturer specific (any fault). 0x1003 keeps the last 8 EMCY codes, newest in sub 1, as error code in bits 0 to 15 and fault number plus the low fault bitmask byte in bits 16 to 31; writing 0 to 0x1003:00 clears it.

## 8. SDO

Expedited transfers only (up to 4 bytes). Segmented and block transfers are answered with abort 0x06010000. Abort codes used: 0x05040001 command specifier, 0x06010000 unsupported access, 0x06010002 read only, 0x06020000 object does not exist, 0x06070010 length mismatch, 0x06090011 sub-index does not exist, 0x06090030 value range, 0x08000021 local control (0x2010), 0x08000022 device state (parameter write outside PRE-OPERATIONAL or during motion).

The lift parameters (0x2100, 0x2101, 0x1400:05) are writable in PRE-OPERATIONAL only and never during motion; a write that makes the configuration inconsistent is refused and reverted. They are volatile: compiled-in defaults return at every reset (no 0x1010 store in v1).

## 9. What this proposal adds or changes against the S2 text

1. F09 for two active stop sensors (no ID in the record).
2. TPDO2 layout (section 4), including velocity and the fault bitmask, so a missed EMCY does not hide the fault ID (S2 gap 1).
3. EMCY codes 0xFF0n and the 0x1003 entry format.
4. RPDO1 deadline armed only during motion (keeps an idle bus quiet).
5. Product code 0x00000011 (17, for CTL-017) and vendor ID 0: there is no CiA vendor ID for the project, TODO(Evans).
6. Own CANopen implementation instead of CANopenNode (D1 names CANopenNode): see docs/ctl017_firmware.md section 4.

## 10. Object table (generated)

Access "rw (PRE-OP only)" is written as `rw` in the EDS; "rww (RPDO only)" is written as `rww`.

<!-- BEGIN GENERATED OD TABLE (tools/ctl017_od/gen_od_docs.py) -->

| Index | Sub | Name | Type | Access | PDO | Default |
|---|---|---|---|---|---|---|
| 0x1000 | 0 | Device type | UNSIGNED32 | const |  | 0x00000000 |
| 0x1001 | 0 | Error register | UNSIGNED8 | ro |  | 0 |
| 0x1003 | 0 | Pre-defined error field: number of errors | UNSIGNED8 | rw |  | 0 |
| 0x1003 | 1 | Standard error field 1 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 2 | Standard error field 2 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 3 | Standard error field 3 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 4 | Standard error field 4 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 5 | Standard error field 5 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 6 | Standard error field 6 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 7 | Standard error field 7 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1003 | 8 | Standard error field 8 | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1014 | 0 | COB-ID EMCY | UNSIGNED32 | const |  | 0x00000090 |
| 0x1016 | 0 | Consumer heartbeat time: highest sub-index | UNSIGNED8 | const |  | 1 |
| 0x1016 | 1 | Consumer heartbeat time 1 (bits 16-23 node-ID, bits 0-15 ms) | UNSIGNED32 | rw |  | 0x0001012C |
| 0x1017 | 0 | Producer heartbeat time (ms) | UNSIGNED16 | rw |  | 0x0064 |
| 0x1018 | 0 | Identity: highest sub-index | UNSIGNED8 | const |  | 4 |
| 0x1018 | 1 | Vendor-ID | UNSIGNED32 | const |  | 0x00000000 |
| 0x1018 | 2 | Product code | UNSIGNED32 | const |  | 0x00000011 |
| 0x1018 | 3 | Revision number | UNSIGNED32 | const |  | 0x00010000 |
| 0x1018 | 4 | Serial number | UNSIGNED32 | ro |  | 0x00000000 |
| 0x1400 | 0 | RPDO1 comm: highest sub-index | UNSIGNED8 | const |  | 5 |
| 0x1400 | 1 | RPDO1 COB-ID | UNSIGNED32 | const |  | 0x00000210 |
| 0x1400 | 2 | RPDO1 transmission type | UNSIGNED8 | const |  | 254 |
| 0x1400 | 5 | RPDO1 event timer: command deadline (ms) | UNSIGNED16 | rw (PRE-OP only) |  | 0x012C |
| 0x1401 | 0 | RPDO2 comm: highest sub-index | UNSIGNED8 | const |  | 5 |
| 0x1401 | 1 | RPDO2 COB-ID | UNSIGNED32 | const |  | 0x00000310 |
| 0x1401 | 2 | RPDO2 transmission type | UNSIGNED8 | const |  | 254 |
| 0x1401 | 5 | RPDO2 event timer (ms, 0 = no deadline) | UNSIGNED16 | const |  | 0x0000 |
| 0x1600 | 0 | RPDO1 mapping: number of entries | UNSIGNED8 | const |  | 3 |
| 0x1600 | 1 | RPDO1 mapping 1 | UNSIGNED32 | const |  | 0x20100108 |
| 0x1600 | 2 | RPDO1 mapping 2 | UNSIGNED32 | const |  | 0x20100208 |
| 0x1600 | 3 | RPDO1 mapping 3 | UNSIGNED32 | const |  | 0x20100308 |
| 0x1601 | 0 | RPDO2 mapping: number of entries | UNSIGNED8 | const |  | 3 |
| 0x1601 | 1 | RPDO2 mapping 1 | UNSIGNED32 | const |  | 0x20110108 |
| 0x1601 | 2 | RPDO2 mapping 2 | UNSIGNED32 | const |  | 0x20110208 |
| 0x1601 | 3 | RPDO2 mapping 3 | UNSIGNED32 | const |  | 0x20110308 |
| 0x1800 | 0 | TPDO1 comm: highest sub-index | UNSIGNED8 | const |  | 5 |
| 0x1800 | 1 | TPDO1 COB-ID | UNSIGNED32 | const |  | 0x40000190 |
| 0x1800 | 2 | TPDO1 transmission type | UNSIGNED8 | const |  | 254 |
| 0x1800 | 3 | TPDO1 inhibit time (100 us) | UNSIGNED16 | rw (PRE-OP only) |  | 0x0064 |
| 0x1800 | 5 | TPDO1 event timer (ms) | UNSIGNED16 | rw |  | 0x0064 |
| 0x1801 | 0 | TPDO2 comm: highest sub-index | UNSIGNED8 | const |  | 5 |
| 0x1801 | 1 | TPDO2 COB-ID | UNSIGNED32 | const |  | 0x40000290 |
| 0x1801 | 2 | TPDO2 transmission type | UNSIGNED8 | const |  | 254 |
| 0x1801 | 3 | TPDO2 inhibit time (100 us) | UNSIGNED16 | rw (PRE-OP only) |  | 0x0064 |
| 0x1801 | 5 | TPDO2 event timer (ms) | UNSIGNED16 | rw |  | 0x0064 |
| 0x1A00 | 0 | TPDO1 mapping: number of entries | UNSIGNED8 | const |  | 7 |
| 0x1A00 | 1 | TPDO1 mapping 1 | UNSIGNED32 | const |  | 0x20000108 |
| 0x1A00 | 2 | TPDO1 mapping 2 | UNSIGNED32 | const |  | 0x20000208 |
| 0x1A00 | 3 | TPDO1 mapping 3 | UNSIGNED32 | const |  | 0x20000310 |
| 0x1A00 | 4 | TPDO1 mapping 4 | UNSIGNED32 | const |  | 0x20000408 |
| 0x1A00 | 5 | TPDO1 mapping 5 | UNSIGNED32 | const |  | 0x20000508 |
| 0x1A00 | 6 | TPDO1 mapping 6 | UNSIGNED32 | const |  | 0x20000608 |
| 0x1A00 | 7 | TPDO1 mapping 7 | UNSIGNED32 | const |  | 0x20000708 |
| 0x1A01 | 0 | TPDO2 mapping: number of entries | UNSIGNED8 | const |  | 6 |
| 0x1A01 | 1 | TPDO2 mapping 1 | UNSIGNED32 | const |  | 0x20010108 |
| 0x1A01 | 2 | TPDO2 mapping 2 | UNSIGNED32 | const |  | 0x20010208 |
| 0x1A01 | 3 | TPDO2 mapping 3 | UNSIGNED32 | const |  | 0x20010310 |
| 0x1A01 | 4 | TPDO2 mapping 4 | UNSIGNED32 | const |  | 0x20010410 |
| 0x1A01 | 5 | TPDO2 mapping 5 | UNSIGNED32 | const |  | 0x20010508 |
| 0x1A01 | 6 | TPDO2 mapping 6 | UNSIGNED32 | const |  | 0x20010608 |
| 0x2000 | 0 | LiftStatus: highest sub-index | UNSIGNED8 | const |  | 7 |
| 0x2000 | 1 | state | UNSIGNED8 | ro | yes | 0 |
| 0x2000 | 2 | flags | UNSIGNED8 | ro | yes | 0 |
| 0x2000 | 3 | position (0.1 mm above lower end, 0xFFFF unknown) | UNSIGNED16 | ro | yes | 0x0000 |
| 0x2000 | 4 | active_stop_id (0xFF none) | UNSIGNED8 | ro | yes | 0 |
| 0x2000 | 5 | last_seq | UNSIGNED8 | ro | yes | 0 |
| 0x2000 | 6 | cmd_result (0xFF none since start-up) | UNSIGNED8 | ro | yes | 0 |
| 0x2000 | 7 | status_counter | UNSIGNED8 | ro | yes | 0 |
| 0x2001 | 0 | LiftStatus2: highest sub-index | UNSIGNED8 | const |  | 6 |
| 0x2001 | 1 | inputs (stop sensors, second E-stop, Hall supply, spares) | UNSIGNED8 | ro | yes | 0 |
| 0x2001 | 2 | target_stop_id (0xFF none) | UNSIGNED8 | ro | yes | 0 |
| 0x2001 | 3 | motor current (mA) | UNSIGNED16 | ro | yes | 0x0000 |
| 0x2001 | 4 | velocity (0.1 mm/s, positive up) | INTEGER16 | ro | yes | 0 |
| 0x2001 | 5 | faults F01..F08 (bit n-1 = Fn) | UNSIGNED8 | ro | yes | 0 |
| 0x2001 | 6 | aux flags (F09, head light on, Getac enable) | UNSIGNED8 | ro | yes | 0 |
| 0x2002 | 0 | Lift fault bitmask (bit n-1 = Fn) | UNSIGNED16 | ro |  | 0x0000 |
| 0x2010 | 0 | LiftCommand: highest sub-index | UNSIGNED8 | const |  | 3 |
| 0x2010 | 1 | seq | UNSIGNED8 | rww (RPDO only) | yes | 0 |
| 0x2010 | 2 | op (0 STOP, 1 MOVE_TO_STOP, 2 CLEAR_FAULT, 3 START_HOMING) | UNSIGNED8 | rww (RPDO only) | yes | 0 |
| 0x2010 | 3 | stop_id (0 L1000, 1 L1175, 2 L1350) | UNSIGNED8 | rww (RPDO only) | yes | 0 |
| 0x2011 | 0 | AuxCommand: highest sub-index | UNSIGNED8 | const |  | 3 |
| 0x2011 | 1 | head light on (0/1) | UNSIGNED8 | rw | yes | 0 |
| 0x2011 | 2 | head light dim level (0..100 %) | UNSIGNED8 | rw | yes | 100 |
| 0x2011 | 3 | Getac power or charging enable (0/1) | UNSIGNED8 | rw | yes | 0 |
| 0x2100 | 0 | Stop positions: highest sub-index | UNSIGNED8 | const |  | 3 |
| 0x2100 | 1 | stop 0 L1000 position (um above lower end) | INTEGER32 | rw (PRE-OP only) |  | 5000 |
| 0x2100 | 2 | stop 1 L1175 position (um above lower end) | INTEGER32 | rw (PRE-OP only) |  | 180000 |
| 0x2100 | 3 | stop 2 L1350 position (um above lower end) | INTEGER32 | rw (PRE-OP only) |  | 355000 |
| 0x2101 | 0 | Lift parameters: highest sub-index | UNSIGNED8 | const |  | 32 |
| 0x2101 | 1 | hall_nm_per_count | INTEGER32 | rw (PRE-OP only) |  | 209200 |
| 0x2101 | 2 | hall_count_up_positive | BOOLEAN | rw (PRE-OP only) |  | 1 |
| 0x2101 | 3 | stop_tol_um | INTEGER32 | rw (PRE-OP only) |  | 2000 |
| 0x2101 | 4 | xcheck_tol_um | INTEGER32 | rw (PRE-OP only) |  | 2000 |
| 0x2101 | 5 | sensor_edge_offset_up_um | INTEGER32 | rw (PRE-OP only) |  | 0 |
| 0x2101 | 6 | sensor_edge_offset_down_um | INTEGER32 | rw (PRE-OP only) |  | 0 |
| 0x2101 | 7 | stop_lead_um | INTEGER32 | rw (PRE-OP only) |  | 0 |
| 0x2101 | 8 | sensor_overrun_um | INTEGER32 | rw (PRE-OP only) |  | 3000 |
| 0x2101 | 9 | duty_move_permille | UNSIGNED16 | rw (PRE-OP only) |  | 0x03E8 |
| 0x2101 | 10 | duty_homing_permille | UNSIGNED16 | rw (PRE-OP only) |  | 0x01F4 |
| 0x2101 | 11 | move_speed_um_s | INTEGER32 | rw (PRE-OP only) |  | 18000 |
| 0x2101 | 12 | homing_speed_um_s | INTEGER32 | rw (PRE-OP only) |  | 9000 |
| 0x2101 | 13 | hall_window_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x00C8 |
| 0x2101 | 14 | hall_start_grace_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x012C |
| 0x2101 | 15 | move_rate_min_cps | UNSIGNED16 | rw (PRE-OP only) |  | 0x002B |
| 0x2101 | 16 | move_rate_max_cps | UNSIGNED16 | rw (PRE-OP only) |  | 0x0082 |
| 0x2101 | 17 | homing_rate_min_cps | UNSIGNED16 | rw (PRE-OP only) |  | 0x000F |
| 0x2101 | 18 | homing_rate_max_cps | UNSIGNED16 | rw (PRE-OP only) |  | 0x0056 |
| 0x2101 | 19 | stall_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x012C |
| 0x2101 | 20 | wrong_dir_counts | UNSIGNED16 | rw (PRE-OP only) |  | 0x000A |
| 0x2101 | 21 | travel_timeout_factor_pct | UNSIGNED16 | rw (PRE-OP only) |  | 0x0096 |
| 0x2101 | 22 | travel_timeout_margin_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x07D0 |
| 0x2101 | 23 | overcurrent_ma | UNSIGNED16 | rw (PRE-OP only) |  | 0x1B58 |
| 0x2101 | 24 | overcurrent_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x0064 |
| 0x2101 | 25 | inrush_blank_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x00C8 |
| 0x2101 | 26 | sensor_debounce_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x0005 |
| 0x2101 | 27 | standstill_quiet_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x0064 |
| 0x2101 | 28 | standstill_timeout_ms | UNSIGNED16 | rw (PRE-OP only) |  | 0x01F4 |
| 0x2101 | 29 | end_switches_wired | BOOLEAN | rw (PRE-OP only) |  | 0 |
| 0x2101 | 30 | homing_continue_to_stop0 | BOOLEAN | rw (PRE-OP only) |  | 1 |
| 0x2101 | 31 | stop_mode_brake | BOOLEAN | rw (PRE-OP only) |  | 0 |
| 0x2101 | 32 | rereference_on_sensor | BOOLEAN | rw (PRE-OP only) |  | 1 |
| 0x2300 | 0 | Diagnostics: highest sub-index | UNSIGNED8 | const |  | 3 |
| 0x2300 | 1 | Hall count (raw) | INTEGER32 | ro |  | 0 |
| 0x2300 | 2 | Reset cause (board port, TODO(Evans) encoding) | UNSIGNED8 | ro |  | 0 |
| 0x2300 | 3 | RPDO length errors | UNSIGNED32 | ro |  | 0x00000000 |

<!-- END GENERATED OD TABLE -->
