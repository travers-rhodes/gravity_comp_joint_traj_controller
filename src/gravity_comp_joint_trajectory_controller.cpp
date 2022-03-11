// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// other imports
#include "ros/ros.h"
#include <ros/console.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

//local package imports
#include <gravity_comp_hardware_interface_adapter.h>

namespace gravity_comp_joint_traj_controller
{
  /**
   * There are two code-injection tricks we use here.
   * The first is to extend a very _specific_ templating of the joint_trajectory_controller
   * which will (internaly) generate a very _specific_ template of HardwareInterfaceAdapter
   * which will be matched to our template pattern in include/gravity_comp_hardware_interface_adapter.h
   * The injected code there creates a HardwareInterfaceAdapter that listens on 
   * HardwareInterfaceAdapter.update_gravity_comp and adds those effort tweaks to all commands
   * before sending effort commands to the robot.
   *
   * The second is that we extend the base JointTrajectoryController class
   * so we can have the JointTrajectoryController.init also start a thread
   * that will compute the required gravity-compensating joint efforts 
   * (and send them to the HardwareInterfaceAdapter)
   */
  class JointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::EffortJointInterface>
  {};
          ;
} 


PLUGINLIB_EXPORT_CLASS(gravity_comp_joint_traj_controller::JointTrajectoryController, controller_interface::ControllerBase)
