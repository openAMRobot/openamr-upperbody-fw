# CTL-017 emulator alignment note (platform-fw #39)

Status: note for the owner of platform-fw #39 (`tools/can_sil/ctl017_emulator.py`, `canopen_common.py`, `ctl017_od_v1.py`). Written against the head of #39 as fetched on 14 Nov. Nothing was pushed to #39. The emulator is protocol evidence, not stop evidence (owner review of 13 Nov).

Target: `docs/ctl017_object_dictionary.md` and `docs/ctl017.eds` on this branch, which are generated from the firmware OD and checked by `od_docs_up_to_date`.

## 1. Object dictionary table (`ctl017_od_v1.py`)

1. **Defaults for integration parameters.** #39 fixes values for 0x1016:01 (65836 = node 1, 300 ms), 0x1017 (100), 0x1018:01..04 (0, 17, 65536, 0), 0x1400:05 (300), 0x1800:03/05 and 0x1801:03/05 (100). On this branch these have no default (TODO(Evans)). The emulator should take them from a config object, refuse to start when one is missing, and print the values it uses. The tests can keep passing the current numbers as **test values**.
2. **0x1400:05 name and meaning.** On this branch it is "RPDO1 event timer (ms, supervised by the lift owner)". The CANopen layer stores it and does not run a deadline. #39 names it "command deadline" and supervises it in the emulator itself (see 2.6).
3. **0x2100 / 0x2101 defaults.** #39 has compiled-in lift parameters. Here they are 0 (not set) until written, and writes go through the lift owner's validator.
4. **Optional permit object.** With `CTL017_PERMIT_OD` the OD gains 0x1402, 0x1602 and a permit RECORD. Index, COB-ID and widths are build parameters with no value (`docs/ctl017_permit_contract.md`). The emulator must not invent them. It should add the RPDO3 only when given the same parameters.
5. Regenerate `ctl017_od_v1.py` from the dump of this branch (`build/tools/ctl017_od/ctl017_od_dump`, which now prints a `param` field), not by hand. `gen_ctl017_od.py` in #39 needs one change: use `param` instead of `value` when it is set.

## 2. Wire behaviour (`canopen_common.py`, `ctl017_emulator.py`)

| # | #39 today | This branch (firmware) | Change in #39 |
|---|---|---|---|
| 2.1 | NMT accepted when `len(data) >= 2` | exactly 2 bytes; unknown CS ignored and counted | `== 2` |
| 2.2 | Heartbeat consumer refreshed by any frame with `len >= 1` from the consumed node | exactly 1 byte with state 0x00/0x04/0x05/0x7F | check length and state |
| 2.3 | Consumer timeout: `supervision_lost = True`, no EMCY | EMCY 0x8130 (error register bit 4, info byte = node), error reset 0x0000 on recovery | emit both |
| 2.4 | Boot-up 0x00 from the base controller treated like any heartbeat | counted as remote boot, event reported | record it so tests can assert it |
| 2.5 | `hbc_config()` on a 0x1016 write clears `supervision_lost` (looks like a recovery) | reconfiguration gives WAITING/OFF with no recovery event | do not clear "lost" as a recovery; go to waiting |
| 2.6 | The emulator runs the lift: deadline, F07, cmd_result, plant | the firmware CANopen layer runs none of this. The lift owner's code does. | keep it in the emulator as a **model of the lift owner's code**, labelled as such, separate from the CANopen part, so a test can tell which layer it exercises |
| 2.7 | NMT 0x82 re-sends boot-up and reconfigures the consumer, but keeps SDO-written 0x1017, PDO timing and 0x1400:05 | restores all of 0x1016:01, 0x1017, 0x1400:05, 0x1800/0x1801:03/05 | restore from the config object |
| 2.8 | RPDO too short: counted only | not processed, EMCY 0x8210 once until a valid RPDO, error register bit 4 | add EMCY and latch |
| 2.9 | RPDO too long: used silently | used; EMCY 0x8220 | add |
| 2.10 | RPDO2 out of range clamped (`min(100, dim)`, `1 if x else 0`) | out of range: not applied, counted, last accepted values kept | do the same |
| 2.11 | TPDO: `event_ms and since >= event_ms` bypasses inhibit | inhibit gates every trigger | `since >= inhibit_ms and (changed or event due)` |
| 2.12 | 0x1016:01 with own node ID accepted | abort 0x06040043 | add |
| 2.13 | SDO frames of any length padded to 8 bytes and served (`ljust(8)`) | SDO requests must be 8 bytes; shorter are dropped | check length |
| 2.14 | Parameter writes refused "during motion" by the emulator | refused outside PRE-OPERATIONAL, and by the lift owner's validator | model the validator as part of the lift-owner model (2.6) |
| 2.15 | Node always configured | an incomplete config keeps the node silent | add a "not configured" start mode to test the master's handling of a silent node |

## 3. Tests to add in #39

- Each row of section 2 as a test, mirrored from `tests/ctl017/test_co_robust.c` (same names, R1 to R11 in `docs/ctl017_canopen_review.md`).
- Cross-check: run the same frame script against the emulator and against `tools/ctl017_sim` (real firmware layers on vcan, fixed status) and compare CANopen-level output: heartbeats, SDO answers, EMCY codes, TPDO timing. Lift-state bytes will differ by design, because the harness has no lift.

## 4. Not to change

- COB-IDs, node-ID 0x10, PDO layouts, EMCY 0xFF0n codes, the change masks 0x7F and 0xC3, and SDO abort codes already match.
- The emulator's lift model (plant, faults) is out of scope for this note. It follows the lift owner's design, not this OD.
