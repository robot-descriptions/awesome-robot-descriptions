# Awesome robot models

[![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

A curated list of awesome robot models in URDF, MJCF or SDF formats.

## Contents

* [Educational](#educational)
* [Humanoids](#humanoids)
* [Manipulators](#manipulators)
* [Quadrupeds](#quadrupeds)
* [Wheeled bipeds](#wheeled-bipeds)

## Educational

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Double pendulum](https://github.com/Gepetto/example-robot-data/tree/master/robots/double_pendulum_description) | URDF | BSD-3-Clause | 2 | 0.7 |
| [FingerEdu v1](https://github.com/Gepetto/example-robot-data/tree/master/robots/finger_edu_description) | URDF | BSD-3-Clause | 3 | 2.3 |
| [Simple Humanoid](https://github.com/laas/simple_humanoid_description) | SDF | BSD-2-Clause | 29 | 131 |

## End effectors

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Allegro hand](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/allegro_hand_description)  | URDF | BSD | 16 | 0.95 |
| [Schunk WSG 50](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/wsg_50_description) | URDF | BSD-3-Clause | 2 | 1.1 |

## Humanoids

_Humanoids have a torso, two legs and/or two arms._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Atlas](https://github.com/RobotLocomotion/drake/tree/master/examples/atlas) | URDF | BSD-3-Clause | 30 | 175 |
| [Baxter](https://github.com/RethinkRobotics/baxter_common/tree/master/baxter_description) | URDF | BSD-3-Clause | 15 | 137 |
| [Bolt](https://github.com/Gepetto/example-robot-data/tree/master/robots/bolt_description) | URDF | BSD-3-Clause | 6 | 1.3 |
| [Cassie](https://github.com/UMich-BipedLab/cassie_description) | URDF | ✖️ | 14 | 32 |
| [iCub](https://github.com/Gepetto/example-robot-data/tree/master/robots/icub_description) | URDF | CC-BY-SA-4.0 | 32 | 28 |
| [JVRC-1](https://github.com/stephane-caron/jvrc_description) | URDF | BSD-2-Clause | 44 | 62 |
| [Reachy](https://github.com/aubrune/reachy_description) | URDF | Apache-2.0 | 21 | 3.2 |
| [Romeo](https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_description) | URDF | BSD-3-Clause | 37 | 41 |
| [TALOS](https://github.com/stack-of-tasks/talos-data) | URDF | LGPL-3.0 | 44 | 109 |
| [TIAGo](https://github.com/Gepetto/example-robot-data/tree/master/robots/tiago_description) | URDF | CC-BY-NC-ND 3.0 | 45 | 66 |

## Manipulators

_Manipulators are robotic arms with a gripper mounted at their end effector._ The list below includes both arms and manipulators.

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Kinova Gen2](https://github.com/Gepetto/example-robot-data/tree/master/robots/kinova_description) | URDF | BSD | 6 | 4.8 |
| [Kinova Jaco](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/jaco_description) | URDF | BSD | 10 | 5.3 |
| [KUKA iiwa 14](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/iiwa_description) | URDF | BSD-3-Clause | 7 | 31 |
| [Panda](https://github.com/Gepetto/example-robot-data/tree/master/robots/panda_description) | URDF | Apache-2.0 | 7 | ✖️ |
| [Kinova Jaco](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/jaco_description) | URDF | BSD | 
| [Universal Robots UR3](https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description) | URDF | BSD | 6 | 11 |
| [Universal Robots UR5](https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description) | URDF | BSD | 6 | 21 |
| [Universal Robots UR10](https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description) | URDF | BSD | 6 | 33 |

## Quadrupeds

_Quadruped robots have four legs._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [A1](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/a1_description) | URDF | MPL-2.0 | 12 | 19 |
| [Aliengo](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description) | URDF | MPL-2.0 | 12 | 21 |
| [ANYmal B](https://github.com/ANYbotics/anymal_b_simple_description) | BSD-3-Clause | URDF | 12 | 30 |
| [ANYmal C](https://github.com/ANYbotics/anymal_c_simple_description) | BSD-3-Clause | URDF | 12 | 52 |
| [Go1](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/laikago_description) | MPL-2.0 | URDF | 12 | 12 |
| [HyQ](https://github.com/Gepetto/example-robot-data/tree/master/robots/hyq_description) | Apache-2.0 | URDF | 12 | 87 |
| [Laikago](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/laikago_description) | MPL-2.0 | URDF | 12 | 25 |
| [Solo](https://github.com/Gepetto/example-robot-data/tree/master/robots/solo_description) | BSD-3-Clause | 12 | 2.5 | 
| [Spot](https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description) | ✖️ | URDF | 12 | ✖️ |

## Wheeled bipeds

_Wheeled bipeds have two legs terminated by wheels._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Upkie](https://github.com/tasts-robots/upkie_description) | URDF | GPL-3.0 | 6 | 5.4 |

## Gallery

| <a href="https://github.com/stephane-caron/jvrc_description"><img src="https://user-images.githubusercontent.com/1189580/161763480-6b2941ad-db98-4f8e-8786-417eefda677e.png" width="100"></a> | <a href="https://github.com/ANYbotics/anymal_b_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755631-3e23d2a5-431f-4b2c-a740-fee92a38a0cd.png" width="100" height="100"></a> | <a href="https://github.com/ANYbotics/anymal_c_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755668-75640c95-f6a9-405f-86bc-590a24ab4db6.png" width="100" height="100"></a> | <a href="https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description"><img src="https://user-images.githubusercontent.com/1189580/161756006-10e81cce-cd7b-4888-a384-4defc902621c.png" width="100" height="100"></a> |
|--|--|--|--|

## Related awesome lists

- [Awesome URDF](https://github.com/ami-iit/awesome-urdf)

## 👷 Contribute

Contributions welcome! Read the [contribution guidelines](CONTRIBUTING.md) first.
