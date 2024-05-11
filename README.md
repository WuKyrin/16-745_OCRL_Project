# Final Project for 16745 Optimal Control

## Project Title

Motion Compensation using MPC for TKA Surgical Robot

## Team Members

Liwei Yang [liweiy@andrew.cmu.edu](liweiy@andrew.cmu.edu)
Qilin Wu [qilinw@andrew.cmu.edu](qilinw@andrew.cmu.edu)

## Objectives

Dynamic compensate motion in TKA surgery

## Installation

Install [tekkneeca](https://github.com/team-paradocs/tekkneeca) in paradocs_ws

Install this repo in ocrl_ws

## Usage

For Gazebo simulation with random pose offset publisher

t1:
```
source both ws
ros2 launch dyna_comp tekkneeca_sim_ocrl.launch.py
```

t2:
```
source ocrl_ws
ros2 launch ocs2_mobile_manipulator_ros manipulator_kuka.launch.py dummy:=true rviz:=true
```

t3:
```
source ocrl_ws
ros2 run ocs2_mobile_manipulator_ros open_loop_mrt_node_torque
```

For physical KUKA LBR MED 7 with random pose offset publisher 

t1:
```
source both ws
ros2 launch dyna_comp tekkneeca_ocrl.launch.py
```

t2:
```
source ocrl_ws
ros2 launch ocs2_mobile_manipulator_ros manipulator_kuka.launch.py dummy:=true rviz:=true
```

t3:
```
source ocrl_ws
ros2 run ocs2_mobile_manipulator_ros open_loop_mrt_node_torque
```

For physical LBR MED 7 with Aruco marker pose

t1:
```
source both ws
ros2 launch dyna_comp tekkneeca_ocrl_aruco.launch.py
```

t2:
source paradocs_ws
```
ros2 launch paradocs_control single.launch.py
subscribe to aruco_result in Rviz
```

t3:
```
source ocrl_ws
ros2 run dyna_comp bone_motion_aruco.py
```

t4:
```
source ocrl_ws
ros2 launch ocs2_mobile_manipulator_ros manipulator_kuka.launch.py
```

t5:
```
source ocrl_ws
ros2 run ocs2_mobile_manipulator_ros open_loop_mrt_node_torque
```
