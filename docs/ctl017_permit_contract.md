# CTL-017 lift permit object: contract proposal

Status: proposal for the CTL-017 owner (Evans) and the platform lead. Agent-written, not hardware-tested, no value decided. Freezing it belongs to I8 and the CTL-017 object dictionary (owner decision of 11 Nov on platform-fw #20).

## 1. What is fixed and what is not

Fixed by the owner decision on platform-fw #20:
- The STM32H723 base controller is the only producer of the lift permit. CTL-017 is the final local gate.
- The permit is false at boot. It is withdrawn at once when a condition is lost, and it is never restored automatically or replayed.
- CTL-017 refuses MOVE_TO_STOP and START_HOMING without a valid permit. While moving it checks that the permit stays fresh, and it stops and drops the target on withdrawal or expiry.
- A heartbeat is not a permit.
- A dedicated cyclic permit PDO is preferred over a bit in RPDO1.

Not fixed. This document names the open items and does not choose for them: COB-ID, mapping and field widths, period, timeout, how a boot or session is identified, sequence rules, reason codes.

What this branch implements: the CANopen layer can receive the permit PDO and show it as plain data (`ctl017_permit_rx_t` in `firmware/ctl017/iface/include/ctl017/lift_iface.h`). It never acts on it. The gate logic (refusing a command, freshness, the stop on withdrawal) belongs to the lift owner and is not written here. The option is off by default.

## 2. Object

One RECORD in the manufacturer area, received by RPDO3 (communication 0x1402, mapping 0x1602), transmission type 0xFE.

| Sub | Field | Type | Meaning (proposal) |
|---|---|---|---|
| 0 | highest sub-index | UNSIGNED8, const | 4 |
| 1 | permit flag | UNSIGNED`CTL017_PERMIT_FLAG_BITS` | 0 = no permit. Which non-zero values mean "permit" is the owner's call (see 4.5). |
| 2 | producer boot identity | UNSIGNED`CTL017_PERMIT_BOOT_BITS` | changes on every base controller boot |
| 3 | sequence | UNSIGNED`CTL017_PERMIT_SEQ_BITS` | +1 per transmission within a boot |
| 4 | reason | UNSIGNED`CTL017_PERMIT_REASON_BITS` | why the permit is missing; 0 = UNKNOWN (I8 rule) |

Access type is `rww`, meaning RPDO only. An SDO write is refused with abort 0x08000021, so no diagnostic tool can inject a permit. 0x1402 and 0x1602 are const. They follow the build parameters and cannot be remapped on the bus.

## 3. Named parameters

| Parameter | Where | Value |
|---|---|---|
| `CTL017_PERMIT_OD_INDEX` | build (`CTL017_PERMIT_DEFS`) | TODO(Evans) |
| `CTL017_PERMIT_COB_ID` | build | TODO(Evans) |
| `CTL017_PERMIT_FLAG_BITS` | build: 8, 16 or 32 | TODO(Evans) |
| `CTL017_PERMIT_BOOT_BITS` | build: 8, 16 or 32 | TODO(Evans) |
| `CTL017_PERMIT_SEQ_BITS` | build: 8, 16 or 32 | TODO(Evans) |
| `CTL017_PERMIT_REASON_BITS` | build: 8, 16 or 32 | TODO(Evans) |
| `permit_event_timer_ms` (0x1402:05) | `ctl017_app_config_t`, runtime | TODO(Evans). Stored and shown to the lift owner. This layer does not supervise it. |
| permit period on the producer | base controller | TODO(platform lead) |

Compile-time checks: the four widths must add up to 64 bits or less. The index must be a free manufacturer index. The COB-ID must lie in 0x081 to 0x57F, which keeps it out of NMT, SYNC, SDO and heartbeat. It must also not be EMCY 0x090 or any PDO of node 0x10. The EMCY IDs of other nodes (0x081 to 0x0FF) are not excluded. Using one of them is the integrator's call. If any parameter is missing while `CTL017_PERMIT_OD=ON`, configuration and compilation stop. The node also stays silent if `permit_event_timer_ms` is 0.

Build example, where the values are placeholders for a test build and not proposals:
`cmake -S . -B build-p -DCTL017_PERMIT_OD=ON -DCTL017_PERMIT_DEFS="CTL017_PERMIT_OD_INDEX=...;CTL017_PERMIT_COB_ID=...;CTL017_PERMIT_FLAG_BITS=...;..."`

## 4. Choices and their consequences

### 4.1 COB-ID
- **A free default-connection-set slot of node 0x10, for example the RPDO3 default.** Consequence: it follows CiA 301 conventions and is easy to read in a capture. It has no priority over LiftCommand (RPDO1 0x210) or below it. At 500 kbit/s with three periodic producers this matters little, but a withdrawal frame then waits behind lower IDs.
- **An ID below the lift command (higher priority).** Consequence: under bus load a withdrawal arrives before any command. The ID must stay above SYNC (the build checks this), and if it lands among the EMCY IDs of other nodes, the bus node list must not use that node, and the master's PDO table on the base controller must be set by hand.
- **The same COB-ID as RPDO1 with a longer frame.** Rejected by the owner decision: a dedicated object is preferred.

### 4.2 Field widths (8 bytes at most)
- Flag 8, boot 16, sequence 16, reason 8 (6 bytes). Consequence: the 16-bit sequence wraps after 65,536 frames, which takes several minutes at any period above a few milliseconds. The consumer has to compare modulo 2^16. A 16-bit boot identity can repeat after 65,536 boots. If it comes from a counter stored in flash, the flash wear on the producer must be checked.
- Flag 8, boot 32, sequence 16, reason 8 (8 bytes). Consequence: this matches the I8 boot_id in full but uses the whole frame, so nothing can be added later without a second PDO.
- Sequence 32 (I8 uses a uint32 sequence). Consequence: no practical wrap, but together with a 32-bit boot identity the flag and reason fields no longer fit.
- An 8-bit sequence. Consequence: it wraps every 256 frames. A consumer that misses more than 255 frames in a row cannot detect the gap.

### 4.3 Boot identity
- **A random value at each boot.** Consequence: it needs no storage. Two boots can produce the same value, with probability 2^-bits.
- **A counter in non-volatile memory.** Consequence: it is unique until it wraps, but it costs a flash write at every boot and needs a policy for when the flash write fails.
- **The I8 `boot_id` itself.** Consequence: the host and CTL-017 see the same identity, so the width must match I8.
Whatever the choice: when the boot identity changes, a permit seen before the restart is no longer valid (owner decision: no automatic restoration).

### 4.4 Period and timeout
- The period is set on the producer. The timeout is set on CTL-017 (the lift owner's freshness check), and 0x1402:05 is offered as the place to configure it. The two are different mechanisms. Neither may reuse the 300 ms heartbeat consumer time or any other timeout (FW sprint rule: timeouts are never merged).
- A short period gives a short timeout and a faster detected loss, at the cost of more bus load and more frames for the master to send.
- A timeout of at least two periods tolerates one lost frame. A timeout of exactly one period turns every lost frame into a withdrawal.
- No stopping time follows from either number. The stopping distance of the lift is measured on the bench.

### 4.5 Flag encoding
- **0 or 1.** Consequence: simple. A single bit flip, which CAN's CRC makes very unlikely, would read as a permit.
- **A fixed non-trivial pattern for "permit" (for example a byte and its complement) and everything else "no permit".** Consequence: random corruption reads as no permit, at the price of a slightly more complex producer.
Whatever the choice: any value other than the agreed "permit" code is no permit.

### 4.6 Reason codes
The base controller knows why the permit is missing (base moving, arm not stowed, fault, inhibit, boot). CTL-017 copies the reason into its status so the operator sees why a command was refused. The table belongs in I8 next to the BaseAuxStatus reason requested by the platform lead. 0 is UNKNOWN.

## 5. Behaviour of the CANopen layer (this branch)

| Event | Effect on `ctl017_permit_rx_t` | Anything else |
|---|---|---|
| Frame with length equal to the mapping, in OPERATIONAL | fields copied raw, `valid = true`, `rx_count` +1, `rx_ms` = receipt time | none |
| Same content again (duplicate) | `rx_count` +1, `rx_ms` updated | none |
| Sequence lower than the previous one | copied as is | none. Judging the order is the lift owner's job. |
| Frame shorter than the mapping | ignored, RPDO length counter +1 | EMCY 0x8210 once (communication error) |
| Frame longer than the mapping | first bytes used | EMCY 0x8220 |
| Not OPERATIONAL | ignored (PDO rule) | none |
| Any NMT transition or reset communication | `valid = false`, content kept for diagnostics | new `nmt_epoch` |
| SDO write to the permit object | refused, abort 0x08000021 | none |
| No frame for any length of time | nothing changes | none. This layer runs no timer on the permit. |

`tests/ctl017/test_permit.c` (built with `CTL017_PERMIT_OD=ON`) checks this table. It also runs the same scenario with and without permit frames and checks that the frames sent and the command records are identical, which shows the layer does not act on the permit.

## 6. Test vectors for the lift owner's gate

These vectors are for the owner's gate code and are not implemented here. Each one starts in OPERATIONAL with the lift idle. Expected results follow the owner decision on platform-fw #20.

| # | Input | Expected |
|---|---|---|
| P1 | MOVE_TO_STOP, no permit frame since boot | refused |
| P2 | permit frame "permit", then MOVE_TO_STOP within the timeout | accepted |
| P3 | permit "permit", wait longer than the timeout, MOVE_TO_STOP | refused (stale) |
| P4 | moving, permit frame "no permit" | stop, target discarded |
| P5 | moving, permit frames stop arriving | stop when the timeout expires, target discarded |
| P6 | moving, boot identity changes and the flag stays "permit" | stop. The new boot's permit is not trusted until (owner rule). |
| P7 | NMT pre-operational then start, last permit was "permit" | refused until a new permit frame arrives |
| P8 | duplicate permit frame (same sequence) | does not refresh freshness (owner rule; choice open) |
| P9 | sequence goes backwards within one boot | treated as no permit (choice open) |
| P10 | permit frame shorter than the mapping | no refresh |
| P11 | after withdrawal, permit becomes "permit" again | no motion until a new command (no replay) |
| P12 | only base controller heartbeats, no permit frames | refused (a heartbeat is not a permit) |
