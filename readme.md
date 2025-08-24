
# Introdution

The architecture is the same as the original [unitree_sdk2](https://github.com/unitreerobotics/unitree_mujoco)

## Features

- Add support for Yahboom USB wireless joystic

  Added joystick mapping code (simulate/src/physics_joystick.h) according to the proxy of [Yahboom USB Wireless Joystick](https://yahboom.com/study_module/PS2)

  Change the `joystick_type` in `simulate/config.yaml` to other values if you want to use joystick of other types