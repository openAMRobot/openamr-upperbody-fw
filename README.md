# openamr-upperbody-fw

Upper-body firmware scope for OpenAMRobot: any required non-vendor end-effector integration, coordinated with the OpenAMRobot 2.0 lift baseline.

> **Status:** Planned; no implementation yet.

The lift is approved in principle for OpenAMRobot 2.0 (P-03 Decision Addendum revision 19.0; DOLD confirmation and release gates per revision 20.2 item 26). Lift control is part of 2.0. The lift is driven by CTL-017, the single upper-body controller: a separate board on base CAN3 (FDCAN3, CANopen 500 kbit/s, node 0x10); the lift H-bridge is not on the base controller (P-03 revision 20.5 item 29, decision D-11; board and pins revision 20.6 item 30 A7). CTL-017 scope: the lift H-bridge and Halls, three SICK IME12-04BNSZC0S stop sensors, the light bar (on only during arm operations), the ZX80 power and charge enable, and spare inputs, including future safety sensors as functional inputs only. Board: NUCLEO-G474RE with BLDC-SHIELD_IFX007T and an SN65HVD230 transceiver, proposed (two units as evaluation stock); Evans confirms the board, the pin map and the input circuit. Non-lift firmware in this repository needs its own agreed scope and acceptance criteria.

## Potential non-lift scope
- **End-effector / gripper controller:** only where the selected device requires custom firmware; use supported vendor integration where available.
- **Upper-body diagnostic or safety-I/O requirements:** ownership and allocation must be agreed with the base-controller and electrical owners. This README does not introduce a separate safety controller or authorize new safety implementation.

## Communication boundaries
- OpenArm uses the kit USB-CAN-FD adapters connected to the Jetson; it does not communicate through an assumed upper-body-to-base serial bridge.
- Base CAN 1 is STM32 ↔ ZLAC8015D, dedicated to traction. Isolated base CAN 2 is STM32 ↔ Daly BMS.
- Base CAN 3 (FDCAN3) is reserved for upper-body auxiliary actuators, CANopen, no safety function; the upper-body controller CTL-017 (node 0x10) is its first node. Lift power runs through the hardwired safety chain.
- Do not attach additional upper-body nodes to base CAN 1 or CAN 2. Any new device requires an agreed CAN interface and allocation. No RS485 or unspecified serial alternative is provisioned.
- Central control electronics remain inside the mobile platform; integral electronics stay with their devices.

## OpenAMRobot 2.0 lift baseline (approved in principle)
- Lift column: DOLD Hexalift V4 350 mm primary, order code BS-HM240G350530-MFF24: 530 mm retracted, 880 mm extended, 11.7 kg catalogue mass, 24 V DC, 800 N push, 18 mm/s, two Hall sensors and two end switches, mounted with its 240 mm side fore-aft. TiMOTION TL3 400 mm fallback.
- Shoulder height 1000 to 1350 mm, measured at the OpenArm arm mount point (`openarm_{side}_base_link`); the upstream OpenArm body is not used.
- Arm mount 0.180 m ahead of the lift column axis, y +/-0.031 m, rpy 0, shoulder spacing 307 mm, 0.166 m above the column top plate.
- Column axis x +0.060 m from `base_link` at `bp000` (330 mm behind the chassis front edge) and x +0.110 m at `bp050`.
- Lift base plate centre or +50 mm (`bp000`, `bp050`); lift stops `L1000`, `L1175`, `L1350`.
- Head camera Stereolabs ZED Mini (ZED-121210), pitch 15 to 35 degrees down in 5 degree steps, baseline 30 degrees (`hp15` to `hp35`, baseline `hp30`); lens 0.060 m ahead of and 0.223 m above the arm mount point (owner decision 8 Nov 2026; was 0.042 m).
- Configuration IDs `column`, `base_plate`, `head_pitch` and `base_tilt` replace `mast_id`.
- Lift control on CAN3 through the upper-body controller CTL-017 (D-11), part of 2.0. Its firmware and ros2_control integration are coordinated with `openamr-platform-fw`, `openamr-upperbody-sw` and `openamr-upperbody-hw` under an approved work package; this README authorizes no implementation.
- Release gates still open: the holding test on the delivered unit after an E-stop during motion and after power loss (DOLD written confirmation of 800 N held with power off received 14 Oct 2026), supplier CAD, weighed mass, the CTL-017 controller with its fixture tests and its CAN message set (base controller firmware owner, I8 work package), and the F2S rerun.

## Lift sensing and safety (decision record, issue #9)
Recorded in [issue #9](https://github.com/openAMRobot/openamr-upperbody-fw/issues/9) (7 October 2026). This README records the decisions and the firmware requirements. No CTL-017 firmware exists yet: nothing below is implemented, and "shall" marks a requirement.
- CTL-017 shall drive the motor stage and read the Hall pulses and both end switches.
- Lift position shall come from the Hall count, referenced to an end switch or a stop sensor.
- Three SICK IME12-04BNSZC0S stop sensors (NPN, normally open, 10 to 30 V; SICK datasheet, M-01 M-09), one per stop `L1000`, `L1175`, `L1350`, on the column rear face (P-03 revision 20.4 item 28). A sensor active at power-up gives a valid position without homing; each stop cross-checks the Hall count. Input stage: **proposed**, a pull-up to 24 V with a divider to 3.3 V, or an optocoupler; the base controller firmware owner decides (issue #14, P-03 revision 20.6 item 30 A7). An open wire reads as "no target", so the firmware shall stop on a travel timeout and shall check the Hall-count window per stop (functional checks, not a safety function).
- CTL-017 shall publish lift position and its validity (no age field; freshness is checked on the receiving side, CAN3 v1). A stale lift state makes the head-camera transform invalid.
- On E-stop, lift power is removed through the hardwired safety chain. The lift holds position with no uncontrolled descent, and its position survives without re-homing.
- The E-stop safe-park procedure includes the lift retracted.
- Lift motion only with the base stopped and the arms in the stowed or carry safe pose.

Part of the OpenAMRobot ecosystem: https://github.com/openAMRobot

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement. See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
