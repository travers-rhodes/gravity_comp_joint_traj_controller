// Copied and modified from https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/hardware_interface_adapter.h
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include "std_msgs/Float32MultiArray.h"

// Because this template is more specific 
// than the templates defined in joint_trajectory_controller,
// the compiler is OK with it and will use it whenever it can.
// One of the times it can use it is in our own code.
// I agree that Templates are confusing and magical, but this way I only need to copypasta
// this single class form joint_trajectory_controller instead of that whole package
template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, 
          joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double> >::State
          >
{
  typedef trajectory_interface::PosVelAccState<double> State;
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) {
  }

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    loop_count_ = 0;
    // add a debug command to publish the latest effort commands
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(controller_nh, "gravity_pid_requested_effort", 1));

    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    // Load velocity feedforward gains from parameter server
    velocity_ff_.resize(joint_handles.size());
    for (unsigned int i = 0; i < velocity_ff_.size(); ++i)
    {
      controller_nh.param(std::string("velocity_ff/") + joint_handles[i].getName(), velocity_ff_[i], 0.0);
    }
   
    // TSR 2022-03-09: Initialize the gravity compensation efforts to zero
    // remember to reset the PID as soon as we receive our first real gravity comp
    // vector (when has_received_real_gravity_comp_ changes to true)
    has_received_real_gravity_comp_ = false;
    gravity_comp_efforts_.resize(joint_handles.size());
    for (unsigned int i = 0; i < gravity_comp_efforts_.size(); ++i)
    {
      gravity_comp_efforts_[i] = 0;
    }

    return true;
  }

  // TSR 2020-03-09: Function to take in recently-computed gravity torques (ie: the compensation needed)
  // efforts (one per joint) and save them 
  // to the offset used in the realtime loop when sending efforts to joints
  void update_gravity_comp(const double gravity_torque[]) {
    // Preconditions
    if (!joint_handles_ptr_)
      return;
    const unsigned int n_joints = joint_handles_ptr_->size();
    assert(n_joints == gravity_comp_efforts_.size());

    // copy the input gravity comp efforts to the ones actually used in the controller
    for (unsigned int i = 0; i < n_joints; ++i) {
      gravity_comp_efforts_[i] = gravity_torque[i];
      ROS_DEBUG_THROTTLE(10,"Joint_%d has gravity %f", i+1, gravity_torque[i]);
    }

    // When we first receive a gravity comp, reset our PIDs and zero our commands
    // to (hopefully) reduce the discontinuity of sudden gravity compensation
    if (!has_received_real_gravity_comp_) {
      // Reset PIDs, zero commands
      for (unsigned int i = 0; i < pids_.size(); ++i)
      {
        pids_[i]->reset();
        (*joint_handles_ptr_)[i].setCommand(0.0);
      }

      has_received_real_gravity_comp_ = true;
    }
  }

  void starting(const ros::Time& /*time*/)
  {
    if (!joint_handles_ptr_) {return;}

    // Reset PIDs, zero commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    // Preconditions
    if (!joint_handles_ptr_)
      return;
    const unsigned int n_joints = joint_handles_ptr_->size();
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());
    

    loop_count_++;
    std::vector<float> debug_efforts_array_;

    // Update PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      // TSR 2022-03-09: Add gravity compensation effort to the command
      // Until we have a real gravity compensation value
      // just send zero commands 
      const double command = has_received_real_gravity_comp_ 
        ? gravity_comp_efforts_[i] + (desired_state.velocity[i] * velocity_ff_[i]) + pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period)
        : 0;
      (*joint_handles_ptr_)[i].setCommand(command);

      // to debug, publish at like 100hz
      if (loop_count_ == 10) {
        debug_efforts_array_.push_back(command);
      }
    }

    // reset the loop counter
    if (loop_count_ == 10) {
      if (realtime_pub_->trylock()){
        realtime_pub_->msg_.data = debug_efforts_array_;
        realtime_pub_->unlockAndPublish();
      }
      loop_count_ = 0;
    }
  }

  bool has_received_real_gravity_comp_;

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<double> velocity_ff_;
  std::vector<double> gravity_comp_efforts_;

  int loop_count_;


  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>> realtime_pub_;
};
