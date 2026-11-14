# CTL-017 CANopen layer: review findings and fixes

Scope: the CANopen slave, application layer and tooling taken from the draft of upperbody-fw #16. Each finding was reviewed against CiA 301 behaviour. A test was written first and the fix came after. The import commit holds the draft files unchanged, so `git diff` against it shows each fix.

| # | Finding in the draft | CiA 301 / reason | Fix | Test |
|---|---|---|---|---|
| R1 | RTR and 29-bit frames were not distinguished: a 29-bit or remote frame with a matching low ID was processed as NMT, SDO or RPDO. `co_frame_t` had no flags. | CANopen objects of this node are 11-bit data frames. RTR on PDOs is not supported (TPDO COB-IDs have bit 30 set). | `ext` and `rtr` flags. Such frames are dropped and counted, as are IDs above 0x7FF and DLC above 8. | `test_rtr_and_extended_dropped` |
| R2 | NMT accepted only DLC 2, but unknown command specifiers were silently ignored and never counted. | NMT node control is exactly 2 bytes. | Wrong length and unknown CS counted as malformed. | `test_nmt_malformed` |
| R3 | The heartbeat consumer accepted any DLC ≥ 1 and any state byte as a heartbeat. A frame of junk refreshed the consumer. | A heartbeat is 1 byte holding 0x00, 0x04, 0x05 or 0x7F. | Only valid heartbeats refresh. | `test_heartbeat_malformed_does_not_refresh` |
| R4 | A heartbeat consumer timeout only called back into the lift core. No EMCY, no error register bit. | A heartbeat event is a communication error: EMCY 0x8130, error register bit 4, error reset 0x0000 when cleared. | Communication error bits in the node, EMCY 0x8130 on timeout, error reset on recovery. Application and communication error-register bits are combined. | `test_heartbeat_consumer_events` |
| R5 | A boot-up (state 0x00) of the monitored node was treated as a normal heartbeat. | A boot-up means the producer restarted. The consumer must be able to see it. | `CO_HBC_EV_REMOTE_BOOT` event and `hbc_remote_boots` counter in the link record. | `test_remote_boot_reported` |
| R6 | Reconfiguring 0x1016:01 while the consumer was LOST reported "heartbeat back" (`lost = false`) without any heartbeat. | A configuration change is not evidence of a live producer. | Reconfiguration goes to WAITING or OFF and clears the comm error. No recovery event. | `test_hbc_reconfig_no_false_recovery` |
| R7 | NMT reset communication re-sent the boot-up but kept the SDO-written 0x1016, 0x1017, PDO inhibit and event times. | Reset communication restores the power-on values of the communication area. | `co_set_comm_defaults()` plus a `reset_comm` callback for the app-owned objects. | `test_reset_comm_restores_defaults` |
| R8 | An RPDO shorter than the mapping was counted but raised no EMCY. A longer one was silently used. | Too short: not processed, EMCY 0x8210. Too long: EMCY 0x8220. | Both added. 0x8210 is latched in the error register until the next valid RPDO. | `test_rpdo_state_and_length` |
| R9 | The TPDO event timer bypassed the inhibit time (`event_ms && since >= event_ms` with no inhibit check). | The inhibit time is the minimum gap between two transmissions of a PDO, whatever triggered them. | The inhibit time gates all triggers. | `test_inhibit_applies_to_event_timer`, `test_tpdo_inhibit_and_change` |
| R10 | The return value of the send callback was ignored. A refused TPDO advanced the status counter and timer, so the frame was lost until the next change. The heartbeat was the same. | Not CiA 301 itself, but a gap the base controller's counter monitor would report as lost frames. | TPDO and heartbeat state advance only on a successful send. Otherwise they retry on the next step and `tx_failures` is counted. | `test_tpdo_send_failure_retried`, `test_heartbeat_retry_after_send_failure` |
| R11 | 0x1016:01 accepted the node's own ID. | A node cannot consume its own heartbeat. | Abort 0x06040043. | `test_sdo_aborts` |

Design findings outside CiA 301, resolved by the interface cut:

| # | Finding | Resolution |
|---|---|---|
| D1 | `cb_rpdo()` sent RPDO1 straight to `lift_command()` (owner review of 13 Nov). | The app layer copies RPDO1 into `ctl017_lift_cmd_rx_t` with a receive counter and time. It never interprets the command. |
| D2 | Timing and identity defaults compiled in (0x1017 100 ms, 0x1016 300 ms, inhibit 10 ms, event 100 ms, product code 0x11, revision 1.0, RPDO1 deadline 300 ms). | `ctl017_app_config_t` has no defaults. An incomplete config gives `CTL017_APP_NOT_CONFIGURED`, and the node sends nothing (`test_not_configured_is_silent`). The EDS and OD table name the parameter instead of a value. |
| D3 | Lift parameters had compiled-in defaults and the app layer checked their consistency. | Storage only, all 0 = not set. A validator supplied by the lift owner decides. With no validator, every write is refused. |
| D4 | The RPDO1 deadline (0x1400:05) was the lift core's command watchdog. | Stored and exposed in the link record. This layer runs no deadline. |
| D5 | Records from before an NMT state change looked current. | `nmt_epoch` counter. Command and permit records are marked not valid on every transition (`test_nmt_epoch_invalidates_records`). |
| D6 | An RPDO2 field out of range was clamped (`min(100, dim)`) and used. | A malformed request is a failure, not a new sample. It is counted, and the last accepted values stay (`test_aux_records`). |

Not changed, open for the owner:
- Segmented SDO stays unsupported. Every v1 object fits in 4 bytes.
- No 0x1010 store. Parameters are volatile.
- Only one heartbeat consumer entry.
- Communication errors other than the two above (CAN controller error passive, bus-off, receive overrun) need the board driver. When it exists they belong in `comm_errors` with EMCY 0x8120, 0x8140 and 0x8110. TODO(Evans).
