# Awesome Robot Models [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

A curated list of awesome robot models in URDF or MJCF formats.

## Contents

* [Robot Models](#robot-models)
    * [Arms](#arms)
    * [Drones](#drones)
    * [Educational](#educational)
    * [End Effectors](#end-effectors)
    * [Humanoids](#humanoids)
    * [Quadrupeds](#quadrupeds)
    * [Wheeled Bipeds](#wheeled-bipeds)
* [Gallery](#gallery)
* [Related Awesome Lists](#related-awesome-lists)
* [Add a Model to the List](#add-a-model-to-the-list)

## Robot Models

### Arms

Robotic arms are serial kinematic chains designed to move an end effector through a workspace.

The list below includes both arms and manipulators (arms with end effectors).

| Name | Maker | Formats | License | Joints | Mass (kg) |
|------|-------|---------|---------|--------|-----------|
| [e.DO](https://github.com/Comau/eDO_description) | Comau | URDF | BSD-3-Clause | 6 | 11 |
| [Gen2](https://github.com/Gepetto/example-robot-data/tree/master/robots/kinova_description) | Kinova | URDF | BSD-3-Clause | 6 | 4.8 |
| [iiwa 14](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/iiwa_description) | KUKA | URDF | BSD-3-Clause | 7 | 31 |
| [Panda](https://github.com/Gepetto/example-robot-data/tree/master/robots/panda_description) | Franka Emika | URDF | Apache-2.0 | 7 | ✖️ |
| [Sawyer](https://github.com/RethinkRobotics/sawyer_robot/tree/master/sawyer_description) | Rethink Robotics | Xacro | Apache-2.0 | 8 | 54 |
| [UR3](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description) | Universal Robots | [Xacro](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/ur3_robot.urdf.xacro), [URDF](https://github.com/Gepetto/example-robot-data/blob/master/robots/ur_description/urdf/ur3_robot.urdf) | Apache-2.0 | 6 | 11 |
| UR5 | Universal Robots | [Xacro](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/ur5_robot.urdf.xacro), [URDF](https://github.com/Gepetto/example-robot-data/blob/master/robots/ur_description/urdf/ur5_robot.urdf) | Apache-2.0 | 6 | 21 |
| UR10 | Universal Robots | [Xacro](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/ur10_robot.urdf.xacro), [URDF](https://github.com/Gepetto/example-robot-data/blob/master/robots/ur_description/urdf/ur10_robot.urdf) | Apache-2.0 | 6 | 33 |

### Drones

Drones, a.k.a. unmanned aerial vehicles (UAVs), are robots that fly in the air.

| Name | Maker | Formats | License | Joints | Mass (kg) |
|------|-------|---------|---------|--------|-----------|
| [Crazyflie 2.0](https://github.com/utiasDSL/gym-pybullet-drones/tree/master/gym_pybullet_drones/assets) | Bitcraze | URDF | MIT | 4 | 0.027 |

### Educational

| Name | Formats | License | Joints | Mass (kg) |
|------|---------|---------|--------|-----------|
| [Double Pendulum](https://github.com/Gepetto/example-robot-data/tree/master/robots/double_pendulum_description) | URDF | BSD-3-Clause | 2 | 0.7 |
| [FingerEdu v1](https://github.com/Gepetto/example-robot-data/tree/master/robots/finger_edu_description) | URDF | BSD-3-Clause | 3 | 2.3 |

### End Effectors

End effectors are sub-systems at the end of a limb, for example a hand at the end of an arm.

| Name | Maker | Formats | License | Joints | Mass (kg) |
|------|-------|---------|---------|--------|-----------|
| [Allegro Hand](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/allegro_hand_description) | Wonik Robotics | URDF | BSD | 16 | 0.95 |
| [WSG 50](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/wsg_50_description) | SCHUNK | SDF | BSD-3-Clause | 2 | 1.1 |

### Humanoids

Humanoids have a torso, two legs and/or two arms.

The list below includes both legged and armed humanoids.

| Name | Maker | Formats | License | Joints | Mass (kg) |
|------|-------|---------|---------|--------|-----------|
| [Atlas](https://github.com/RobotLocomotion/drake/tree/master/examples/atlas) | Boston Dynamics | URDF | BSD-3-Clause | 30 | 175 |
| [Baxter](https://github.com/RethinkRobotics/baxter_common/tree/master/baxter_description) | Rethink Robotics | URDF, Xacro | BSD-3-Clause | 15 | 137 |
| [Bolt](https://github.com/Gepetto/example-robot-data/tree/master/robots/bolt_description) | ODRI | URDF | BSD-3-Clause | 6 | 1.3 |
| [Cassie](https://github.com/UMich-BipedLab/cassie_description) | Agility Robotics | URDF | ✖️ | 14 | 32 |
| [iCub](https://github.com/Gepetto/example-robot-data/tree/master/robots/icub_description) | IIT | URDF | CC-BY-SA-4.0 | 32 | 28 |
| [JVRC-1](https://github.com/stephane-caron/jvrc_description) | AIST | URDF | BSD-2-Clause | 44 | 62 |
| [Reachy](https://github.com/aubrune/reachy_description) | Pollen Robotics | URDF | Apache-2.0 | 21 | 3.2 |
| [PR2](https://github.com/PR2/pr2_common/tree/melodic-devel/pr2_description) | Willow Garage | Xacro, [URDF](https://github.com/RobotLocomotion/drake) | BSD | 14 | 227 |
| [Romeo](https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_description) | Aldebaran Robotics | URDF | BSD-3-Clause | 37 | 41 |
| [Simple Humanoid](https://github.com/laas/simple_humanoid_description) | N/A | URDF | BSD-2-Clause | 29 | 131 |
| [TALOS](https://github.com/stack-of-tasks/talos-data) | PAL Robotics | URDF | LGPL-3.0 | 44 | 109 |
| [TIAGo](https://github.com/Gepetto/example-robot-data/tree/master/robots/tiago_description) | PAL Robotics | URDF | CC-BY-NC-ND 3.0 | 45 | 66 |
| [WALK-MAN](https://github.com/ADVRHumanoids/iit-walkman-ros-pkg/tree/master/walkman_urdf) | IIT | Xacro | BSD-3-Clause | 30 | 94 |

### Quadrupeds

Quadruped robots have four legs.

| Name | Maker | Formats | License | Joints | Mass (kg) |
|------|-------|---------|---------|--------|-----------|
| [A1](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/a1_description) | UNITREE Robotics | URDF | MPL-2.0 | 12 | 19 |
| [Aliengo](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description) | UNITREE Robotics | URDF | MPL-2.0 | 12 | 21 |
| [ANYmal B](https://github.com/ANYbotics/anymal_b_simple_description) | ANYbotics | URDF | BSD-3-Clause | 12 | 30 |
| [ANYmal C](https://github.com/ANYbotics/anymal_c_simple_description) | ANYbotics | URDF | BSD-3-Clause | 12 | 52 |
| [HyQ](https://github.com/Gepetto/example-robot-data/tree/master/robots/hyq_description) | IIT | URDF | Apache-2.0 | 12 | 87 |
| [Laikago](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/laikago_description) | UNITREE Robotics | URDF | MPL-2.0 | 12 | 25 |
| [Mini Cheetah](https://github.com/graiola/wolf_descriptions/tree/master/minicheetah_description) | MIT | Xacro | BSD | 12 | 9.0 |
| [Solo](https://github.com/Gepetto/example-robot-data/tree/master/robots/solo_description) | ODRI | URDF | BSD-3-Clause | 12 | 2.5 |
| [Spot](https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description) | Boston Dynamics | URDF | ✖️ | 12 | ✖️ |

### Wheeled Bipeds

Wheeled bipeds have two legs terminated by wheels.

| Name | Maker | Formats | License | Joints | Mass (kg) |
|------|-------|---------|---------|--------|-----------|
| [Upkie](https://github.com/tasts-robots/upkie_description) | Tast's Robots | URDF | Apache-2.0 | 6 | 5.4 |

## Gallery

| <a href="https://github.com/Comau/eDO_description"><img src="https://user-images.githubusercontent.com/1189580/186620995-72a1bf23-5b01-43a8-a259-b0554b104bde.png" width=100></a> | <a href="https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description"><img src="https://user-images.githubusercontent.com/1189580/186625328-0a90663e-8250-4558-a621-b87c69513d06.png" width=100></a> | |
|--|--|--|
| <a href="https://github.com/utiasDSL/gym-pybullet-drones/tree/master/gym_pybullet_drones/assets"><img src="https://user-images.githubusercontent.com/1189580/184339424-e392b662-3191-4a9a-83fd-d3ad1d0cc992.png" width=100></a> | | |
| <a href="https://github.com/Gepetto/example-robot-data/tree/master/robots/bolt_description"><img src="https://user-images.githubusercontent.com/1189580/172120044-9f3fc7fb-7082-4b81-b3f4-a10b4d5593b3.png" width=100></a> | <a href="https://github.com/stephane-caron/jvrc_description"><img src="https://user-images.githubusercontent.com/1189580/161763480-6b2941ad-db98-4f8e-8786-417eefda677e.png" width=100></a> | <a href="https://github.com/tasts-robots/upkie_description"><img src="https://user-images.githubusercontent.com/1189580/169592756-0d0f00a8-4adf-487c-a4fd-85a82b7f6ad1.png" width=100></a> | |
| <a href="https://github.com/ANYbotics/anymal_b_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755631-3e23d2a5-431f-4b2c-a740-fee92a38a0cd.png" width=100></a> | <a href="https://github.com/ANYbotics/anymal_c_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755668-75640c95-f6a9-405f-86bc-590a24ab4db6.png" width=100></a> | <a href="https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description"><img src="https://user-images.githubusercontent.com/1189580/161756006-10e81cce-cd7b-4888-a384-4defc902621c.png" width=100></a> |

## Related Awesome Lists

* [Awesome Homemade Robots](https://github.com/tasts-robots/awesome-homemade-robots)
* [Awesome URDF](https://github.com/ami-iit/awesome-urdf)

## Add a Model to the List

New models are welcome! Check out the [guidelines](CONTRIBUTING.md), then open a PR.
