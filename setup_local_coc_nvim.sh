catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
catkin build
ln -s ../../build/gravity_comp_joint_traj_controller/compile_commands.json compile_commands.json
