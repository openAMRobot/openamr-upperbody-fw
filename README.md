# openamr-upperbody-fw

Upper-body firmware scope for OpenAMRobot: any required non-vendor end-effector integration and the future OpenAMRobot 3.0 lift controller.

> **Status:** Planned; no implementation yet.

OpenAMRobot 2.0 uses a fixed mast. Lift-controller implementation and lift requirements belong to OpenAMRobot 3.0. This deferral does not set the schedule for non-lift firmware; any such work needs its own agreed scope and acceptance criteria.

## Potential non-lift scope
- **End-effector / gripper controller:** only where the selected device requires custom firmware; use supported vendor integration where available.
- **Upper-body diagnostic or safety-I/O requirements:** ownership and allocation must be agreed with the base-controller and electrical owners. This README does not introduce a separate safety controller or authorize new safety implementation.

## Communication boundaries
- OpenArm uses the kit USB-CAN-FD adapters connected to the Jetson; it does not communicate through an assumed upper-body-to-base serial bridge.
- Base CAN 1 is STM32 ↔ ZLAC8015D, dedicated to traction. Isolated base CAN 2 is STM32 ↔ Daly BMS.
- Do not attach additional upper-body nodes to either dedicated base bus. Any new device requires an agreed CAN interface and allocation. No RS485 or unspecified serial alternative is provisioned.
- Central control electronics remain inside the mobile platform; integral electronics stay with their devices.

## OpenAMRobot 3.0 lift scope
Lift requirements include travel, payload, speed and safety requirements. Future lift firmware and its ros2_control integration belong to a separately approved work package coordinated with `openamr-upperbody-sw` and `openamr-upperbody-hw`. No lift controller is part of the OpenAMRobot 2.0 fixed-mast baseline.

Part of the OpenAMRobot ecosystem: https://github.com/openAMRobot

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement. See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
