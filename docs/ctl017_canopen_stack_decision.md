# CTL-017 CANopen stack: own slave or CANopenNode (decision D1)

Status: recommendation for the CTL-017 owner (Evans). This is not a decision. D1 named CANopenNode. The draft (upperbody-fw #16) used its own slave instead, and the owner review of 13 Nov flagged that as a departure from D1. This page gives the owner the facts to choose.

## 1. What CTL-017 needs from the stack

| Feature | Needed | Own slave (`firmware/ctl017/canopen`) | CANopenNode (CiA 301 part) |
|---|---|---|---|
| NMT slave, boot-up, states | yes | yes | yes |
| Heartbeat producer 0x1017 | yes | yes | yes |
| Heartbeat consumer 0x1016, one node | yes | yes, one entry, events reported | yes, many entries, callbacks |
| Expedited SDO server | yes | yes | yes |
| Segmented / block SDO | no for v1 (all objects ≤ 4 bytes) | no, abort 0x06010000 | yes (segmented; block optional) |
| 2 TPDO, 2 RPDO, static mapping | yes | yes, plus RPDO3 with the permit option | yes, static or dynamic |
| PDO inhibit and event timer | yes | yes | yes |
| RPDO length check with EMCY 0x8210/0x8220 | yes (CiA 301) | yes (fixed on this branch) | yes |
| RPDO deadline monitoring (event timer) | the lift owner decides | stored and exposed only | yes, inside the stack (`CO_RPDO` timeout) |
| EMCY producer, 0x1001, 0x1003 | yes | yes | yes |
| SYNC, TIME, LSS, node guarding, SDO client, LEDs | no | no | yes (each can be compiled out) |
| Store parameters 0x1010 | not in v1 | no | yes (storage layer) |
| EDS from the firmware | yes | generator from the built OD, checked by ctest | OD is generated **from** an EDS/XDD by CANopenEditor |

## 2. Code size (measured, Cortex-M4, `-Os`, GCC 13 arm-none-eabi)

| Build | text bytes | Notes |
|---|---|---|
| Own slave `co_node.c` | 2,784 | MinSizeRel; this branch |
| Own app + OD (`ctl017_app.c`, `ctl017_od.c`, `lift_pdo.c`) | 8,782 | the OD is built at run time from a table in code |
| Own total | ~11.6 k | no static RAM in the library; one `ctl017_app_t` instance in the caller |
| CANopenNode 301 objects, example OD, blank driver | 14,482 text + 976 data | `CANopen.c` plus EMCY, HB consumer, NMT, OD interface, PDO, SDO server, SYNC, TIME, LEDs; default `CO_config.h` |
| CANopenNode with SYNC, TIME, LEDs compiled out | about 12.6 k (subtraction of the measured objects) | not built; estimate from the per-object numbers |

Method: CANopenNode master at commit 9b8beed (10 Jul 2026) cloned to a scratch directory. Each source file listed above was compiled with `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -ffunction-sections -fdata-sections` and measured with `arm-none-eabi-size`. The CTL-017 OD is not generated for CANopenNode, so its size would differ from the example OD. The STM32G474RE has 512 KB of flash, so neither option is constrained by size.

## 3. Licence and how to take it in

CANopenNode is Apache-2.0. The project is MIT. Apache-2.0 code can be used in an MIT project as long as the licence and NOTICE stay with the code and modified files are marked.

Proposal: fetch, do not vendor.
- CMake `FetchContent` (or a git submodule) pinned to a commit hash, with `FetchContent_Declare(... GIT_TAG <hash>)`. The licence file arrives with the sources, and the repository contains no third-party code to keep in sync.
- The CubeIDE board build (when the owner writes it) needs a local copy. A script under `tools/` that clones the pinned commit into an ignored directory keeps the CubeIDE and CMake builds on the same sources.
- Driver glue (`CO_driver_target.h` and a G4 FDCAN driver) is our own code under MIT. The community STM32 drivers (CanOpenSTM32) are a separate repository whose licence has to be checked when the driver is chosen. TODO(Evans).
- Offline builds (CI without network) need a cache or a mirror. A CI job that fetches at configure time breaks when GitHub is unreachable.

## 4. Test reuse

| Test | With own slave | With CANopenNode |
|---|---|---|
| `test_canopen` (wire behaviour through the app layer) | as is | reusable. It only uses frames in and out, except the checks that read `F.app.co.*` fields, which would read the CANopenNode objects instead. |
| `test_co_robust` (malformed frames, consumer events, reset comm) | as is | reusable as a conformance check of our configuration of CANopenNode. Some assertions encode our choices (for example `rx_malformed` counters), and CANopenNode has no matching counter. |
| `test_permit` | as is | reusable, with the permit mapped in the CANopenNode OD |
| vcan pytest | as is | as is: it only sees the bus |
| `od_docs_up_to_date` | generator reads the built OD | reversed: the EDS becomes the source and the C OD is generated from it. The check then compares the CANopenEditor output with the committed files. |
| Lift test double and iface | unchanged | unchanged: the iface does not depend on the stack |

## 5. Migration cost (estimate, not measured)

1. G4 FDCAN driver for CANopenNode (`CO_driver_target.h`, receive buffers with filters, error counters, bus-off): the largest part. With our own slave this driver is also needed, in a smaller form (two callbacks).
2. CTL-017 OD in CANopenEditor, generated `OD.c/OD.h`, and the extension hooks (`OD_extension_init`) for 0x2010/0x2011/permit to fill the iface records.
3. App layer rewritten against `CO_t` instead of `co_node_t`. The iface and the lift owner's code stay unchanged.
4. Decisions that CANopenNode makes differently and must be configured or accepted: RPDO timeout handling inside the stack (it raises EMCY 0x8250 and an error status bit, and the lift owner's code must still own the reaction), the heartbeat consumer with several entries, and SDO segmented transfer enabled by default.
5. Tests: adapt the internal-field checks. The wire-level tests carry over.

## 6. Recommendation (for the owner to decide)

- **Keep CANopenNode as D1 says if** the project wants one stack across CTL-017 and the base controller (the STM32H723 master side is also CANopen), long-term maintenance by an upstream community, or later features (0x1010 store, segmented SDO for strings, LSS). The licence handling above is routine.
- **Keep the own slave if** v1 stays at this fixed, small object set and the team prefers about 940 lines of slave code (source and header) that every reviewer can read fully. The costs: CiA 301 corner cases are ours to find (this branch fixed the eleven listed in docs/ctl017_canopen_review.md), and D1 must be formally amended.
- The interface boundary on this branch (`firmware/ctl017/iface`) makes the choice reversible. The lift owner's code does not see which stack is used.

Agent view: the deciding factor is the base controller. If the STM32H723 uses CANopenNode for CAN1 and CAN3 anyway, using it on CTL-017 too halves what the team has to know. If the base controller uses something else, the own slave is cheaper to own for v1. TODO(Evans): decide D1 and record it in the CAN3 decision list.
