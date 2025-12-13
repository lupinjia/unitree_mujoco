
# Introdution

The architecture is the same as the original [unitree_sdk2](https://github.com/unitreerobotics/unitree_mujoco)

## Additional Dependencies

- [opencv 4.12.0](https://github.com/opencv/opencv/tree/4.12.0)

## Features

- Add support for Yahboom USB wireless joystick (C++)

  Added joystick mapping code (simulate/src/physics_joystick.h) according to the proxy of [Yahboom USB Wireless Joystick](https://yahboom.com/study_module/PS2)

  Change the `joystick_type` in `simulate/config.yaml` to other values if you want to use joystick of other types

- Add support for depth image accessing through both Python and C++ API

  Through calling mujoco API and adding camera element in `go2.xml` we can obtain the depth image from mujoco.

  The processed depth image is published to a dds topic named "rt/depthimage" using cyclonedds (currently only in Python).

- Press `Enter` to track base_link of the robot, press `Esc` to use free camera