// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// imports for kdl gravity computation loop
#include <boost/scoped_ptr.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Float32.h"
#include <thread>

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
  {
    public:
      // Gravity compensation calculations were
      // heavily inspired by https://github.com/Learning-and-Intelligent-Systems/mit-ros-pkg
      bool init(hardware_interface::EffortJointInterface* hw,
                ros::NodeHandle&   root_nh,
                ros::NodeHandle&   controller_nh) {
        // TSR 2022-06-24 Lol. Wondering why I had so much network traffic. Don't automatically set to Debug for logger.
        // https://www.theconstructsim.com/ros-qa-141-how-to-modify-logger-level-in-ros-c/
        //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        //{
        //  ros::console::notifyLoggerLevelsChanged();
        //}
        // call the base class's init method
        ROS_DEBUG("Calling init method");
        bool success = joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
            hardware_interface::EffortJointInterface>::init(hw, root_nh, controller_nh);

        if (!success) {
          return false;
        }

        // pulling relevant kinematics strings from parameter server
        ROS_DEBUG("Getting kinematics parameters");
        std::string root_name, tip_name, robot_description;
        if (!root_nh.getParam("root_name", root_name))
        {
          ROS_ERROR("No root name given in namespace: %s)",
              root_nh.getNamespace().c_str());
          return false;
        }
        if (!root_nh.getParam("tip_name", tip_name))
        {
          ROS_ERROR("No tip name given in namespace: %s)",
              root_nh.getNamespace().c_str());
          return false;
        }
        if (!root_nh.getParam("robot_description", robot_description))
        {
          ROS_ERROR("No robot_description given in namespace: %s)",
              root_nh.getNamespace().c_str());
          return false;
        }
        double default_gravity_loop_rate = 500;
        if (!root_nh.getParam("gravity_loop_rate", gravity_loop_rate_))
        {
          ROS_WARN("No gravity_loop_rate given in namespace: %s, using %0.1fHz)",
              root_nh.getNamespace().c_str(), default_gravity_loop_rate);
	  gravity_loop_rate_ = default_gravity_loop_rate;
        }

        ROS_DEBUG("Initializing KDL chain");
        // Construct kdl_chain_ parameter first
        if (!constructKDLChain(root_name, tip_name, robot_description, kdl_chain_)) {
          ROS_ERROR("Couldn't construct chain from %s to %s.)",
              root_name.c_str(), tip_name.c_str());
          return false;
        }

        ROS_DEBUG("Initializing KDL chain with gravity");
        // Construct the kdl_chain_dyn_param_ using the kdl_chain_ and the gravity vector
        grav_vector_ = KDL::Vector(0., 0., -9.81);
        kdl_chain_dyn_param_.reset(new KDL::ChainDynParam(kdl_chain_, grav_vector_));
        // initialize the jnt_gravity_ parameter 
        // (which we update/repeatedly copy to the HardwareInterfaceAdapter
        // in order to give the gravity torques we need to account for)
        jnt_gravity_.resize(kdl_chain_.getNrOfJoints());
        q_.resize(kdl_chain_.getNrOfJoints());
        ROS_DEBUG("Hardware Adapter Interface Initialization Complete");
      
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(controller_nh, "gravity_loop_rate", 4));

        // start a thread to constantly update the gravity compensation values
        // and detach from it so it keeps on running in the background
        update_gravity_loop_thread_ = std::thread(&JointTrajectoryController::calculate_and_update_gravity_compensation, this);
        update_gravity_loop_thread_.detach();
        return success;
      }
    private:
      // This function should be called in a background thread
      // It is just a nice loop to use current joint positions
      // to compute gravity on each joint
      // and then pass those up to the HardwareInterfaceAdapter
      // so that they are added to the effort commands as gravity compensation
      void calculate_and_update_gravity_compensation() {
        int loop_count_mod_thousand = 0;
        ros::Time thousand_loops_start = ros::Time::now();
        ros::Time thousand_loops_end = ros::Time::now();
        ros::Rate gravity_loop_rate(gravity_loop_rate_);
        while (ros::ok()) {
          for (int i = 0; i < this->joints_.size(); i++)
          {
            q_(i) = this->joints_[i].getPosition();
          }
          kdl_chain_dyn_param_->JntToGravity(q_, jnt_gravity_);
          // jnt_gravity_ is a JntArray. jnt_gravity_.data is an Eigen::VectorXd
          // jnt_gravity_.data.data() is a double[] 
          this->hw_iface_adapter_.update_gravity_comp(jnt_gravity_.data.data());
          gravity_loop_rate.sleep();
          loop_count_mod_thousand++;
          if (loop_count_mod_thousand == 1000) {
            loop_count_mod_thousand = 0;
            thousand_loops_end = ros::Time::now();
            if (realtime_pub_->trylock()){
              realtime_pub_->msg_.data = 1000/(thousand_loops_end-thousand_loops_start).toSec();
              realtime_pub_->unlockAndPublish();
            }
            thousand_loops_start = ros::Time::now();
          }
        }
      }

      // COPIED FROM pr2_mechanism_model::Chain
      // fills out kdl_chain_ variable and joints_ variable
      bool constructKDLChain(std::string root, std::string tip, std::string robot_description, KDL::Chain& kdl_chain) {
        // Constructs the kdl chain
        KDL::Tree kdl_tree;
        // kdl_parser from http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
        if (!kdl_parser::treeFromString(robot_description, kdl_tree)){
          ROS_ERROR("Could not convert robot_description string into kdl tree");
          return false;
        }
      
        bool res;
        try{
          res = kdl_tree.getChain(root, tip, kdl_chain);
        }
        catch(...){
          res = false;
        }
        if (!res){
          ROS_ERROR("Could not extract chain between %s and %s from kdl tree",
              root.c_str(), tip.c_str());
          return false;
        }
        ROS_DEBUG("Loaded %d segments into the KDL chain", kdl_chain.getNrOfSegments());
        for (int i = 0; i < kdl_chain.getNrOfSegments(); i++) {
          ROS_DEBUG("Segment %d has mass %f", i, kdl_chain.getSegment(i).getInertia().getMass());
          ROS_DEBUG("Segment %d has COG: (%f,%f,%f)", 
              i, 
              kdl_chain.getSegment(i).getInertia().getCOG()[0],
              kdl_chain.getSegment(i).getInertia().getCOG()[1],
              kdl_chain.getSegment(i).getInertia().getCOG()[2]);
        }
        ROS_DEBUG("Loaded %d joints into the KDL chain", kdl_chain.getNrOfJoints());
      
        return true;
      }

      // Create an empty thread here, but we save it to the object
      // so that when the object is destroyed the thread is terminated
      std::thread update_gravity_loop_thread_;
      double gravity_loop_rate_;
      boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32>> realtime_pub_;

      // Referenced in initialization
      // gravity, relative to KDL chain base link
      KDL::Vector grav_vector_; 
      /// The chain of links and joints in KDL language
      // Read-only after initialization
      KDL::Chain kdl_chain_;
      // The chain of links combined with the gravity vector for gravity comp calculations
      boost::scoped_ptr<KDL::ChainDynParam> kdl_chain_dyn_param_;

      // Referenced only in update loop
      // Joint positions
      KDL::JntArray  q_; 
      // Gravity torque on each joint
      KDL::JntArray  jnt_gravity_; 
  };
} 


PLUGINLIB_EXPORT_CLASS(gravity_comp_joint_traj_controller::JointTrajectoryController, controller_interface::ControllerBase)
