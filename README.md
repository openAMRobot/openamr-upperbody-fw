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
