# Joint Trajectory Controller with Gravity Compensation

This package is in beta and has only so far been developed for Kinova Gen3 7DOF robot.

# Setup

This package relies on `https://github.com/travers-rhodes/kinova_gen3_control`. Follow installation instructions there.

Compile using something like
```
catkin config --blacklist kortex_control kortex_driver kortex_examples gazebo_version_helpers gazebo_grasp_plugin roboticsgroup_gazebo_plugins gen3_lite_gen3_lite_2f_move_it_config gen3_move_it_config gen3_robotiq_2f_140_move_it_config roboticsgroup_upatras_gazebo_plugins gen3_robotiq_2f_85_move_it_config kortex_move_it_config
```

# Example Usage

After compilation alongside `kinova_gen3_control`, you can test this package in simulation or on the real robot.

### Gazebo Simulation

To test in simulation, first run:
```
roslaunch gravity_comp_joint_traj_controller test_gazebo.launch
```
and then call the following in a terminal window (in which you have not forgotten to `source devel/setup.bash`). The robot will jiggle around the target (because no friction) but should move to be jiggling near the target position within 10 seconds.

```
rostopic pub /my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names:
    - 'joint_1'
    - 'joint_2'
    - 'joint_3'
    - 'joint_4'
    - 'joint_5'
    - 'joint_6'
    - 'joint_7'
    points:
    - positions: [0.18, 1.34, 2.19,-0.60,2.20,-0.60,-0.08]
      velocities: [0,0,0,0,0,0,0]
      accelerations: [0,0,0,0,0,0,0]
      effort: [0,0,0,0,0,0,0]
      time_from_start: {secs: 10, nsecs: 0}
  path_tolerance:
  - {name: '', position: 3.0, velocity: 3.0, acceleration: 3.0}
  goal_tolerance:
  - {name: '', position: 0.2, velocity: 0.2, acceleration: 0.2}
  goal_time_tolerance: {secs: 10, nsecs: 0}" 
```

To convince yourself gravity comp really is working, you can call something like the following to change the force of gravity to 0 in Gazebo. The robot will move up since it's compensating for gravity that is no longer there.
```
rosservice call /gazebo/set_physics_properties "time_step: 0.001
max_update_rate: 1000.0
gravity:
  x: 0.0
  y: 0.0
  z: 0.0
ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50,
  sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0,
  cfm: 0.0, erp: 0.2, max_contacts: 20}" 
```

### Testing on Real Robot
To test on the real robot, run
```
roslaunch gravity_comp_joint_traj_controller default.launch fake_connection:=false
```
and then call (note the slight difference in path---in Gazebo the path starts with `/my_gen3/` but here it does not).
```
rostopic pub /gen3_joint_trajectory_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names:
    - 'joint_1'
    - 'joint_2'
    - 'joint_3'
    - 'joint_4'
    - 'joint_5'
    - 'joint_6'
    - 'joint_7'
    points:
    - positions: [0.18, 1.34, 2.19,-0.60,2.20,-0.60,-0.08]
      velocities: [0,0,0,0,0,0,0]
      accelerations: [0,0,0,0,0,0,0]
      effort: [0,0,0,0,0,0,0]
      time_from_start: {secs: 10, nsecs: 0}
  path_tolerance:
  - {name: '', position: 3.0, velocity: 3.0, acceleration: 3.0}
  goal_tolerance:
  - {name: '', position: 0.2, velocity: 0.2, acceleration: 0.2}
  goal_time_tolerance: {secs: 10, nsecs: 0}" 
```


