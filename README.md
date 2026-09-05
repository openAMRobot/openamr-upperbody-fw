# openamr-upperbody-fw

Upper-body firmware for the OpenAMRobot mobile manipulator: microcontroller code for the lift and end-effector mounted on the mobile base.

> **Status:** Planned, no code yet

Populated next cycle, after the lift mechanical and electrical concept is set. This repository currently holds only this README.

## What will live here
- **Lift controller:** homing, position control, soft and hard travel limits.
- **End-effector / gripper controller:** when the end-effector is not a vendor unit.
- **Upper-body safety I/O:** interlocks for motion at height.

## Interfaces
- Talks to the mobile base over the defined power and CAN or serial link owned by `openamr-platform-hw` and its firmware bridge.
- Exposes the lift to ros2_control in `openamr-upperbody-sw`.

## This cycle
Requirements only (lift travel, payload, speed, safety), handed from `openamr-upperbody-sw` to the hardware concept. Firmware development starts next cycle.

Part of the OpenAMRobot ecosystem: https://github.com/openAMRobot

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement. See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
