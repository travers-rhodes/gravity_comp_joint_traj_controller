// Just used to make (fake) EffortControlInterface
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <kinova_gen3_control/kinova_gen3_hardware_interface.h>
#include "kinova_gen3_control/fake_kinova_network_connection.h"
#include "controller_manager/controller_manager.h"

// other imports
#include "ros/ros.h"
#include <ros/console.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testing");
  ros::NodeHandle nh;
  ros::NodeHandle cm_node(nh, "/gen3_joint_trajectory_controller");

  // we need an async spinner to look for and handle start/stop service calls to the controller_manager 
  // while the main thread is in its inner loop computing update messages to send to the robot joints
  ros::AsyncSpinner spinner(1);
  spinner.start();


  std::shared_ptr<KinovaNetworkConnection> network_connection;
  network_connection = std::make_shared<FakeKinovaNetworkConnection>(); 

  ROS_INFO("Creating hardware interface");
  std::vector<std::string> joint_names = {
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7"
  };

  urdf::Model urdf_robot;
  // load urdf from "robot_description"
  // TODO don't hard-code this string
  urdf_robot.initParam("robot_description");

  std::vector<joint_limits_interface::JointLimits> limits_list;
  for (int i = 0; i < joint_names.size(); i++)
  {
    joint_limits_interface::JointLimits limits;
    std::shared_ptr<const urdf::Joint> urdf_joint = urdf_robot.getJoint(joint_names[i]);
    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(urdf_joint, limits);
    // Populate joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' 
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = joint_limits_interface::getJointLimits(joint_names[i], nh, limits);
    limits_list.push_back(limits);
  }

  KinovaGen3HardwareInterface robot(
      joint_names,
      limits_list,
      network_connection);
  
  ROS_INFO("Starting controller manager");
  controller_manager::ControllerManager controller_manager(&robot, nh);
  ROS_INFO("Controller manager started");

  ROS_WARN("hello\n");
  HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, 
          joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double> >::State
      > hw_iface_adapter;
  ros::Rate r(10);
  ros::Time current = ros::Time::now();
  ros::Time previous = ros::Time::now();
  ros::Duration controller_manager_loop_duration;
  // https://www.theconstructsim.com/ros-qa-141-how-to-modify-logger-level-in-ros-c/
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  while (ros::ok())
  {
    robot.read();
    current = ros::Time::now();
    controller_manager_loop_duration = current-previous;
    robot.write(controller_manager_loop_duration);
    controller_manager.update(current, controller_manager_loop_duration);
    r.sleep();
  }

  return 0;
}
