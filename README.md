# Awesome Robot Descriptions [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

A curated list of awesome robot descriptions in URDF or MJCF formats.

## Contents

* [Robot Descriptions](#robot-descriptions)
    * [Arms](#arms)
    * [Bipeds](#bipeds)
    * [Dual arms](#dual-arms)
    * [Drones](#drones)
    * [Educational](#educational)
    * [End Effectors](#end-effectors)
    * [Mobile Manipulators](#mobile-manipulators)
    * [Humanoids](#humanoids)
    * [Quadrupeds](#quadrupeds)
* [Gallery](#gallery)
* [Related Awesome Lists](#related-awesome-lists)
* [Add a Description to the List](#add-a-description-to-the-list)

## Robot Descriptions

### Arms

Robotic arms are serial kinematic chains designed to move an end effector through a workspace.

The list below includes both arms and manipulators (arms with end effectors).

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| e.DO | Comau | [URDF](https://github.com/Comau/eDO_description) | BSD-3-Clause | 6 | ✔️ | ✔️ |
| Gen2 | Kinova | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/kinova_description) | BSD-3-Clause | 6 | ✔️ | ✔️ |
| iiwa 14 | KUKA | [URDF](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/iiwa_description) | BSD-3-Clause | 7 | ✔️ | ✔️ |
| Panda (MJCF) | Franka Emika | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/franka_emika_panda) | Apache-2.0 | 7 | ✔️ | ✔️ |
| Panda (URDF) | Franka Emika | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/panda_description), [Xacro](https://github.com/frankaemika/franka_ros/tree/develop/franka_description) | Apache-2.0 | 7 | ✖️ | ✔️ |
| Sawyer | Rethink Robotics | [Xacro](https://github.com/RethinkRobotics/sawyer_robot/tree/master/sawyer_description) | Apache-2.0 | 8 | ✔️ | ✔️ |
| UR 3/5/10 | Universal Robots | [URDF](https://github.com/Gepetto/example-robot-data/blob/master/robots/ur_description/), [Xacro](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/) | Apache-2.0 | 6 | ✔️ | ✔️ |
| URe 3/5/10 | Universal Robots | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e), [URDF](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_e_description) | BSD-3-Clause | 6 | ✔️ | ✔️ |

### Bipeds

Bipeds have two legs terminated by feet or wheels.

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| Bolt | ODRI | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/bolt_description) | BSD-3-Clause | 6 | ✔️ | ✔️ |
| Cassie (MJCF) | Agility Robotics | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/agility_cassie) | MIT | 20 | ✔️ | ✔️ |
| Cassie (URDF) | Agility Robotics | [URDF](https://github.com/UMich-BipedLab/cassie_description) | ✖️ | 14 | ✔️ | ✔️ |
| Upkie | Tast's Robots | [URDF](https://github.com/tasts-robots/upkie_description) | Apache-2.0 | 6 | ✔️ | ✔️ |

### Dual arms

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| Baxter | Rethink Robotics | [URDF](https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_description/urdf/baxter.urdf), [Xacro](https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_description/urdf/baxter.urdf.xacro) | BSD-3-Clause | 15 | ✔️ | ✔️ |
| PR2 | Willow Garage | [URDF](https://github.com/ankurhanda/robot-assets/tree/master/urdfs/robots/pr2), [Xacro](https://github.com/PR2/pr2_common/blob/melodic-devel/pr2_description/urdf/common.xacro) | BSD | 14 | ✔️ | ✔️ |
| Reachy | Pollen Robotics | [URDF](https://github.com/aubrune/reachy_description) | Apache-2.0 | 21 | ✔️ | ✔️ |
| YuMi | ABB | [URDF](https://github.com/OrebroUniversity/yumi/tree/master/yumi_description) | BSD-2-Clause | 16 | ✔️ | ✔️ |

### Drones

Drones, a.k.a. unmanned aerial vehicles (UAVs), are robots that fly in the air.

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| Crazyflie 2.0 | Bitcraze | [URDF](https://github.com/utiasDSL/gym-pybullet-drones/tree/master/gym_pybullet_drones/assets) | MIT | 4 | ✔️ | ✔️ |

### Educational

| Name | Formats | License | Joints | Inertias | Collisions |
|------|---------|---------|--------|----------|------------|
| Double Pendulum | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/double_pendulum_description) | BSD-3-Clause | 2 | ✔️ | ✔️ |
| FingerEdu | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/finger_edu_description) | BSD-3-Clause | 3 | ✔️ | ✔️ |
| Simple Humanoid | [URDF](https://github.com/laas/simple_humanoid_description) | BSD-2-Clause | 29 | ✔️ | ✖️ |

### End Effectors

End effectors are sub-systems at the end of a limb, for example a hand at the end of an arm.

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| Allegro Hand | Wonik Robotics | [URDF](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/allegro_hand_description) | BSD | ✔️ | ✔️ |
| Robotiq 2F-85 | Robotiq | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/robotiq_2f85), [URDF](https://github.com/a-price/robotiq_arg85_description), [Xacro](https://github.com/ros-industrial/robotiq/tree/kinetic-devel/robotiq_2f_85_gripper_visualization) | BSD-2-Clause | 8 | ✔️ | ✔️ |
| Shadow Hand E3M5 | The Shadow Robot Company | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/shadow_hand) | Apache-2.0 | 20 | ✔️ | ✔️ |
| WSG 50 | SCHUNK | [SDF](https://github.com/RobotLocomotion/drake/tree/master/manipulation/models/wsg_50_description) | BSD-3-Clause | 2 | ✔️ | ✔️ |

### Mobile Manipulators

Mobile manipulators have a wheeled mobile base and one or two arms.

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| Fetch | Fetch Robotics | [URDF](https://github.com/openai/roboschool/tree/master/roboschool/models_robot/fetch_description) | MIT | 14 | ✔️ | ✔️ |
| Reachy | Pollen Robotics | [URDF](https://github.com/aubrune/reachy_description) | Apache-2.0 | 21 | ✔️ | ✔️ |
| PR2 | Willow Garage | [URDF](https://github.com/ankurhanda/robot-assets/tree/master/urdfs/robots/pr2), [Xacro](https://github.com/PR2/pr2_common/blob/melodic-devel/pr2_description/urdf/common.xacro) | BSD | 14 | ✔️ | ✔️ |
| TIAGo | PAL Robotics | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/tiago_description) | CC-BY-NC-ND 3.0 | 45 | ✔️ | ✔️ |

### Humanoids

Humanoids have a torso, two legs and two arms.

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| Atlas DRC (v3) | Boston Dynamics | [URDF](https://github.com/RobotLocomotion/drake/tree/master/examples/atlas) | BSD-3-Clause | 30 | ✔️ | ✔️ |
| Atlas v4 | Boston Dynamics | [URDF](https://github.com/openai/roboschool/tree/1.0.49/roboschool/models_robot/atlas_description) | MIT | 30 | ✔️ | ✔️ |
| iCub | IIT | [URDF](https://github.com/robotology/icub-models/tree/master/iCub) | CC-BY-SA-4.0 | 32 | ✔️ | ✔️ |
| JVRC-1 | AIST | [MJCF](https://github.com/isri-aist/jvrc_mj_description/), [URDF](https://github.com/stephane-caron/jvrc_description) | BSD-2-Clause | 44 | ✔️ | ✔️ |
| Romeo | Aldebaran Robotics | [URDF](https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_description) | BSD-3-Clause | 37 | ✔️ | ✔️ |
| TALOS | PAL Robotics | [URDF](https://github.com/stack-of-tasks/talos-data) | LGPL-3.0 | 44 | ✔️ | ✔️ |
| WALK-MAN | IIT | [Xacro](https://github.com/ADVRHumanoids/iit-walkman-ros-pkg/tree/master/walkman_urdf) | BSD-3-Clause | 30 | ✔️ | ✔️ |

### Quadrupeds

Quadruped robots have four legs.

| Name | Maker | Formats | License | Joints | Inertias | Collisions |
|------|-------|---------|---------|--------|----------|------------|
| A1 | UNITREE Robotics | [MJCF](https://github.com/unitreerobotics/unitree_mujoco/tree/main/data/a1/xml), [URDF](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/a1_description) | MPL-2.0 | 12 | ✔️ | ✔️ |
| Aliengo | UNITREE Robotics | [MJCF](https://github.com/unitreerobotics/unitree_mujoco/tree/main/data/aliengo/xml), [URDF](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description) | MPL-2.0 | 12 | ✔️ | ✔️ |
| ANYmal B | ANYbotics | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/anybotics_anymal_b), [URDF](https://github.com/ANYbotics/anymal_b_simple_description) | BSD-3-Clause | 12 | ✔️ | ✔️ |
| ANYmal C | ANYbotics | [MJCF](https://github.com/deepmind/mujoco_menagerie/tree/main/anybotics_anymal_c), [URDF](https://github.com/ANYbotics/anymal_c_simple_description) | BSD-3-Clause | 12 | ✔️ | ✔️ |
| Go1 | UNITREE Robotics | [MJCF](https://github.com/unitreerobotics/unitree_mujoco/tree/main/data/go1/xml), [URDF](https://github.com/unitreerobotics/unitree_mujoco/tree/main/data/go1/urdf) | BSD-3-Clause | 12 | ✔️ | ✔️ |
| HyQ | IIT | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/hyq_description) | Apache-2.0 | 12 | ✔️ | ✔️ |
| Laikago | UNITREE Robotics | [MJCF](https://github.com/unitreerobotics/unitree_mujoco/tree/main/data/laikago/xml), [URDF](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/laikago_description) | MPL-2.0 | 12 | ✔️ | ✔️ |
| Mini Cheetah | MIT | [URDF](https://github.com/Derek-TH-Wang/mini_cheetah_urdf) | BSD | 12 | ✔️ | ✔️ |
| Minitaur | Ghost Robotics | [URDF](https://github.com/bulletphysics/bullet3/blob/master/data/quadruped/minitaur.urdf) | BSD-2-Clause | 12 | ✔️ | ✔️ |
| Solo | ODRI | [URDF](https://github.com/Gepetto/example-robot-data/tree/master/robots/solo_description) | BSD-3-Clause | 12 | ✔️ | ✔️ |
| Spot | Boston Dynamics | [Xacro](https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description) | ✖️ | 12 | ✖️ | ✔️ |

## Gallery

| <a href="https://github.com/Comau/eDO_description"><img src="https://user-images.githubusercontent.com/1189580/186620995-72a1bf23-5b01-43a8-a259-b0554b104bde.png" width=100></a> | <a href="https://github.com/Gepetto/example-robot-data/tree/master/robots/ur_description"><img src="https://user-images.githubusercontent.com/1189580/186625328-0a90663e-8250-4558-a621-b87c69513d06.png" width=100></a> | | |
|--|--|--|--|
| <a href="https://github.com/Gepetto/example-robot-data/tree/master/robots/bolt_description"><img src="https://user-images.githubusercontent.com/1189580/172120044-9f3fc7fb-7082-4b81-b3f4-a10b4d5593b3.png" alt="Bolt" width=100></a> | <a href="https://github.com/UMich-BipedLab/cassie_description/"><img src="https://user-images.githubusercontent.com/1189580/187714165-2ac847c7-36c2-4960-aca4-acd7064a6fad.png" alt="Cassie" width=100></a> | <a href="https://github.com/tasts-robots/upkie_description"><img src="https://user-images.githubusercontent.com/1189580/169592756-0d0f00a8-4adf-487c-a4fd-85a82b7f6ad1.png" alt="Upkie" width=100></a> | |
| <a href="https://github.com/utiasDSL/gym-pybullet-drones/tree/master/gym_pybullet_drones/assets"><img src="https://user-images.githubusercontent.com/1189580/187918557-d819641b-9b54-46ae-a9a0-8e2f0c73e01b.png" alt="Crazyflie 2.0" width=100></a> | | | |
| <a href="https://github.com/aubrune/reachy_description"><img src="https://user-images.githubusercontent.com/1189580/187716758-f6cf526c-f555-4e0b-9f7c-49762846a7da.png" alt="Reachy" width=100></a> | | | |
| <a href="https://github.com/RobotLocomotion/drake/tree/master/examples/atlas"><img src="https://user-images.githubusercontent.com/1189580/187859356-9e1ae5c6-7bf6-4cf2-96f1-3171510ae9cf.png" alt="Atlas" width=100></a> | <a href="https://github.com/stephane-caron/jvrc_description"><img src="https://user-images.githubusercontent.com/1189580/161763480-6b2941ad-db98-4f8e-8786-417eefda677e.png" alt="JVRC-1" width=100></a> | <a href="https://github.com/ros-aldebaran/romeo_robot/"><img src="https://user-images.githubusercontent.com/1189580/187856976-44a32a33-a6af-4888-a080-7d270ed88b86.png" alt="Romeo" width=100></a> | <a href="https://github.com/laas/simple_humanoid_description"><img src="https://user-images.githubusercontent.com/1189580/187855410-e37602c5-5e44-42cb-9379-92326c46e945.png" alt="Simple Humanoid" width=100></a> | |
| <a href="https://github.com/unitreerobotics/unitree_ros/tree/master/robots/a1_description/"><img src="https://user-images.githubusercontent.com/1189580/188139145-0d1c49e4-84d2-43a7-852d-35073f6fe516.png" alt="A1" width=100></a> | <a href="https://github.com/ANYbotics/anymal_b_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755631-3e23d2a5-431f-4b2c-a740-fee92a38a0cd.png" alt="ANYmal B" width=100></a> | <a href="https://github.com/ANYbotics/anymal_c_simple_description"><img src="https://user-images.githubusercontent.com/1189580/161755668-75640c95-f6a9-405f-86bc-590a24ab4db6.png" alt="ANYmal C" width=100></a> | <a href="https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description"><img src="https://user-images.githubusercontent.com/1189580/161756006-10e81cce-cd7b-4888-a384-4defc902621c.png" alt="Spot" width=100></a> |

## Related Awesome Lists

* [Awesome Homemade Robots](https://github.com/tasts-robots/awesome-homemade-robots)
* [Awesome URDF](https://github.com/ami-iit/awesome-urdf)

## Add a Description to the List

New robot descriptions are welcome! Check out the [guidelines](CONTRIBUTING.md), then open a PR.
