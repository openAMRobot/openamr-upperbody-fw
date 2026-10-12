# openamr-upperbody-fw

Upper-body firmware scope for OpenAMRobot: any required non-vendor end-effector integration, coordinated with the OpenAMRobot 2.0 lift baseline.

> **Status:** Planned; no implementation yet.

The lift is approved in principle for OpenAMRobot 2.0 (P-03 Decision Addendum revision 19.0). Lift control is part of 2.0. The STM32H723ZG base controller (NUCLEO-H723ZG) commands the lift through a small CAN-to-lift interface module, an MCU or ready-made module acting as a CAN node on CAN3, through the generic actuator interface of the firmware core. Non-lift firmware in this repository needs its own agreed scope and acceptance criteria.

## Potential non-lift scope
- **End-effector / gripper controller:** only where the selected device requires custom firmware; use supported vendor integration where available.
- **Upper-body diagnostic or safety-I/O requirements:** ownership and allocation must be agreed with the base-controller and electrical owners. This README does not introduce a separate safety controller or authorize new safety implementation.

## Communication boundaries
- OpenArm uses the kit USB-CAN-FD adapters connected to the Jetson; it does not communicate through an assumed upper-body-to-base serial bridge.
- Base CAN 1 is STM32 ↔ ZLAC8015D, dedicated to traction. Isolated base CAN 2 is STM32 ↔ Daly BMS.
- Base CAN 3 (FDCAN3) is reserved for upper-body auxiliary actuators, CANopen, no safety function; the lift interface module is its first node. Lift power runs through the hardwired safety chain.
- Do not attach additional upper-body nodes to base CAN 1 or CAN 2. Any new device requires an agreed CAN interface and allocation. No RS485 or unspecified serial alternative is provisioned.
- Central control electronics remain inside the mobile platform; integral electronics stay with their devices.

## OpenAMRobot 2.0 lift baseline (approved in principle)
- Lift column: DOLD Hexalift V4 350 mm primary, order code BS-HM240G350530-MFF24: 530 mm retracted, 880 mm extended, 11.7 kg catalogue mass, 24 V DC, two Hall sensors and two end switches. TiMOTION TL3 400 mm fallback.
- Shoulder height 1000 to 1350 mm, measured at the OpenArm arm mount point (`openarm_{side}_base_link`); the upstream OpenArm body is not used.
- Arm mount 0.180 m ahead of the lift column axis, y +/-0.031 m, rpy 0, shoulder spacing 307 mm, 0.166 m above the column top plate.
- Column axis x +0.060 m from `base_link` at `bp000` (330 mm behind the chassis front edge) and x +0.110 m at `bp050`.
- Lift base plate centre or +50 mm (`bp000`, `bp050`); lift stops `L1000`, `L1175`, `L1350`.
- Head camera Stereolabs ZED Mini (ZED-121210), pitch 15 to 35 degrees down in 5 degree steps, baseline 25 degrees (`hp15` to `hp35`).
- Configuration IDs `column`, `base_plate`, `head_pitch` and `base_tilt` replace `mast_id`.
- Lift control on CAN3 through the CAN-to-lift interface module and the STM32H723ZG base controller, part of 2.0. Its firmware and ros2_control integration are coordinated with `openamr-platform-fw`, `openamr-upperbody-sw` and `openamr-upperbody-hw` under an approved work package; this README authorizes no implementation.
- Release gates still open: DOLD holding confirmation on E-stop and power loss, supplier CAD, F2S rerun, CAN-to-lift module selection and its CAN message set (Evans).

## Lift sensing and safety (decision record, issue #9)
Recorded in [issue #9](https://github.com/openAMRobot/openamr-upperbody-fw/issues/9) (7 October 2026). This README records the decisions; it implements no firmware or safety function.
- The CAN-to-lift interface module drives the motor stage and reads the Hall pulses and both end switches.
- Lift position comes from the Hall count, referenced to an end switch.
- The module publishes lift position, validity and age. A stale lift state makes the head-camera transform invalid.
- On E-stop, lift power is removed through the hardwired safety chain. The lift holds position with no uncontrolled descent, and its position survives without re-homing.
- The E-stop safe-park procedure includes the lift retracted.
- Lift motion only with the base stopped and the arms in the stowed or carry safe pose.

Part of the OpenAMRobot ecosystem: https://github.com/openAMRobot

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement. See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
