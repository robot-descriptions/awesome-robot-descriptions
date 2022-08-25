# Awesome Robot Models [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

A curated list of awesome robot models in URDF or MJCF formats.

## Contents

* [Robot Models](#robot-models)
    * [Arms](#arms)
    * [Drones](#drones)
    * [Educational](#educational)
    * [End effectors](#end-effectors)
    * [Humanoids](#humanoids)
    * [Quadrupeds](#quadrupeds)
    * [Wheeled bipeds](#wheeled-bipeds)
* [Gallery](#gallery)
* [Related Awesome Lists](#related-awesome-lists)
* [Add a Model to the List](#add-a-model-to-the-list)

## Robot Models

### Arms

_Robotic arms are serial kinematic chains designed to move an end effector through a workspace._

The list below includes both arms and manipulators (arms with end effectors).

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [e.DO](https://github.com/Comau/eDO_description) | URDF | BSD-3-Clause | 6 | 11 |
| [Kinova Gen2](https://github.com/Gepetto/example-robot-data/tree/master/robots/kinova_description) | URDF | BSD-3-Clause | 6 | 4.8 |
| [Kinova Jaco](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/jaco_description) | URDF | BSD-3-Clause | 10 | 5.3 |
| [KUKA iiwa 14](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/iiwa_description) | URDF | BSD-3-Clause | 7 | 31 |
| [Panda](https://github.com/Gepetto/example-robot-data/tree/master/robots/panda_description) | URDF | Apache-2.0 | 7 | ✖️ |
| [Sawyer](https://github.com/RethinkRobotics/sawyer_robot/tree/master/sawyer_description) | Xacro/URDF | Apache-2.0 | 8 | 54 |
| [Universal Robots UR3](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description) | Xacro/URDF | Apache-2.0 | 6 | 11 |
| [Universal Robots UR5](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description) | Xacro/URDF | Apache-2.0 | 6 | 21 |
| [Universal Robots UR10](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description) | Xacro/URDF | Apache-2.0 | 6 | 33 |

### Drones

_Drones, a.k.a. unmanned aerial vehicles (UAVs), are robots that fly in the air._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Crazyflie 2.0](https://github.com/utiasDSL/gym-pybullet-drones/tree/master/gym_pybullet_drones/assets) | URDF | MIT | 4 | 0.027 |

### Educational

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Double Pendulum](https://github.com/Gepetto/example-robot-data/tree/master/robots/double_pendulum_description) | URDF | BSD-3-Clause | 2 | 0.7 |
| [FingerEdu v1](https://github.com/Gepetto/example-robot-data/tree/master/robots/finger_edu_description) | URDF | BSD-3-Clause | 3 | 2.3 |

### End effectors

_End effectors are sub-systems at the end of a limb, for example a hand at the end of an arm._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Allegro Hand](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/allegro_hand_description)  | URDF | BSD | 16 | 0.95 |
| [Schunk WSG 50](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/wsg_50_description) | URDF | BSD-3-Clause | 2 | 1.1 |

### Humanoids

_Humanoids have a torso, two legs and/or two arms._

The list below includes both legged and armed humanoids.

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
| [Simple Humanoid](https://github.com/laas/simple_humanoid_description) | URDF | BSD-2-Clause | 29 | 131 |
| [TALOS](https://github.com/stack-of-tasks/talos-data) | URDF | LGPL-3.0 | 44 | 109 |
| [TIAGo](https://github.com/Gepetto/example-robot-data/tree/master/robots/tiago_description) | URDF | CC-BY-NC-ND 3.0 | 45 | 66 |
| [WALK-MAN](https://github.com/ADVRHumanoids/iit-walkman-ros-pkg/tree/master/walkman_urdf) | Xacro/URDF | BSD-3-Clause | 30 | 94 |

### Quadrupeds

_Quadruped robots have four legs._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [A1](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/a1_description) | URDF | MPL-2.0 | 12 | 19 |
| [Aliengo](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description) | URDF | MPL-2.0 | 12 | 21 |
| [ANYmal B](https://github.com/ANYbotics/anymal_b_simple_description) | URDF | BSD-3-Clause | 12 | 30 |
| [ANYmal C](https://github.com/ANYbotics/anymal_c_simple_description) | URDF | BSD-3-Clause | 12 | 52 |
| [HyQ](https://github.com/Gepetto/example-robot-data/tree/master/robots/hyq_description) | URDF | Apache-2.0 | 12 | 87 |
| [Laikago](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/laikago_description) | URDF | MPL-2.0 | 12 | 25 |
| [MIT Mini Cheetah](https://github.com/graiola/wolf_descriptions/tree/master/minicheetah_description) | Xacro/URDF | BSD | 12 | 9.0 |
| [Solo](https://github.com/Gepetto/example-robot-data/tree/master/robots/solo_description) | URDF | BSD-3-Clause | 12 | 2.5 |
| [Spot](https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description) | URDF | ✖️ | 12 | ✖️ |

### Wheeled bipeds

_Wheeled bipeds have two legs terminated by wheels._

| Name | Format | License | Joints | Mass (kg) |
|------|--------|---------|--------|-----------|
| [Upkie](https://github.com/tasts-robots/upkie_description) | URDF | Apache-2.0 | 6 | 5.4 |

## Gallery

| <a href="https://github.com/Comau/eDO_description"><img src="https://user-images.githubusercontent.com/1189580/186620995-72a1bf23-5b01-43a8-a259-b0554b104bde.png" width=100></a> | | |
|--|--|--|
| <a href="https://github.com/utiasDSL/gym-pybullet-drones/tree/master/gym_pybullet_drones/assets"><img src="https://user-images.githubusercontent.com/1189580/184339424-e392b662-3191-4a9a-83fd-d3ad1d0cc992.png" width=100></a> | | |
| <a href="https://github.com/Gepetto/example-robot-data/tree/master/robots/bolt_description"><img src="https://user-images.githubusercontent.com/1189580/172120044-9f3fc7fb-7082-4b81-b3f4-a10b4d5593b3.png" width=100></a> | <a href="https://github.com/stephane-caron/jvrc_description"><img src="https://user-images.githubusercontent.com/1189580/161763480-6b2941ad-db98-4f8e-8786-417eefda677e.png" width=100></a> | <a href="https://github.com/tasts-robots/upkie_description"><img src="https://user-images.githubusercontent.com/1189580/169592756-0d0f00a8-4adf-487c-a4fd-85a82b7f6ad1.png" width=100></a> | |
| <a href="https://github.com/ANYbotics/anymal_b_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755631-3e23d2a5-431f-4b2c-a740-fee92a38a0cd.png" width=100></a> | <a href="https://github.com/ANYbotics/anymal_c_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755668-75640c95-f6a9-405f-86bc-590a24ab4db6.png" width=100></a> | <a href="https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description"><img src="https://user-images.githubusercontent.com/1189580/161756006-10e81cce-cd7b-4888-a384-4defc902621c.png" width=100></a> |

## Related Awesome Lists

* [Awesome Homemade Robots](https://github.com/tasts-robots/awesome-homemade-robots)
* [Awesome URDF](https://github.com/ami-iit/awesome-urdf)

## Add a Model to the List

New models are welcome! Check out the [guidelines](CONTRIBUTING.md) before opening a PR.
