#include <joint_velocity_interface.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace simple_franka_interface {

bool JointVelocityInterface::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  sub_joint_velocity_target_ = node_handle.subscribe(
    "joint_velocity_target", 20, &JointVelocityInterface::jointVelCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_joint_angle_target_ = node_handle.subscribe(
    "joint_angle_target", 20, &JointVelocityInterface::jointAngCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());
      
  pub_joint_state_ = node_handle.advertise<geometry_msgs::PoseStamped>("joint_state", 1, true);
  pub_end_state_ = node_handle.advertise<geometry_msgs::PoseStamped>("end_state", 1, true);

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointVelocityInterface: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointVelocityInterface: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }
  return true;
}

void JointVelocityInterface::starting(const ros::Time& /* time */) {

  for (size_t i = 0; i < 7; ++i) {
    joint_angle_target_[i] = velocity_joint_handles_[i].getPosition();
    joint_velocity_target_[i] = 0;
  }

}

void JointVelocityInterface::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  statePublish();
  for (size_t i = 0; i < 7; ++i) {
    //PI control
    if (abs(joint_velocity_target_[i]) > 3.5){
      joint_velocity_target_[i] = 0.1*(joint_velocity_target_[i]/abs(joint_velocity_target_[i]));
    }
    velocity_joint_handles_[i].setCommand(joint_velocity_target_[i] + filter_params_*(joint_angle_target_[i] - velocity_joint_handles_[i].getPosition()));
  }
}

//Subscribe joint vlocity reference.
//Up to down, joint 0 to joint 6.
void JointVelocityInterface::jointVelCallback(const geometry_msgs::PoseStamped& msg) {
  joint_velocity_target_[0] = msg.pose.orientation.w;
  joint_velocity_target_[1] = msg.pose.orientation.x;
  joint_velocity_target_[2] = msg.pose.orientation.y;
  joint_velocity_target_[3] = msg.pose.orientation.z;
  joint_velocity_target_[4] = msg.pose.position.x;
  joint_velocity_target_[5] = msg.pose.position.y;
  joint_velocity_target_[6] = msg.pose.position.z;
}

//Subscribe joint anlge reference.
//Up to down, joint 0 to joint 6.
void JointVelocityInterface::jointAngCallback(const geometry_msgs::PoseStamped& msg) {
  joint_angle_target_[0] = msg.pose.orientation.w;
  joint_angle_target_[1] = msg.pose.orientation.x;
  joint_angle_target_[2] = msg.pose.orientation.y;
  joint_angle_target_[3] = msg.pose.orientation.z;
  joint_angle_target_[4] = msg.pose.position.x;
  joint_angle_target_[5] = msg.pose.position.y;
  joint_angle_target_[6] = msg.pose.position.z;
}

void JointVelocityInterface::statePublish(){

  //Publish end pose.
  geometry_msgs::PoseStamped pose_msg;
  std::array<double, 16> end_pose = state_handle_->getRobotState().O_T_EE;
  pose_msg.pose.position.x = end_pose[12];
  pose_msg.pose.position.y = end_pose[13];
  pose_msg.pose.position.z = end_pose[14];
  Eigen::Matrix3d end_orientation; 
  end_orientation<< 
  end_pose[0],end_pose[4],end_pose[8],
  end_pose[1],end_pose[5],end_pose[9],
  end_pose[2],end_pose[6],end_pose[10];
  Eigen::Quaterniond end_quaternion(end_orientation);
  pose_msg.pose.orientation.x = end_quaternion.x();
  pose_msg.pose.orientation.y = end_quaternion.y();
  pose_msg.pose.orientation.z = end_quaternion.z();
  pose_msg.pose.orientation.w = end_quaternion.w();

  pub_end_state_.publish(pose_msg);

  //Publish joint angle.
  //Up to down, joint 0 to joint 6.
  geometry_msgs::PoseStamped joint_angle_state_msg;
  joint_angle_state_msg.pose.orientation.w = velocity_joint_handles_[0].getPosition();
	joint_angle_state_msg.pose.orientation.x = velocity_joint_handles_[1].getPosition();
	joint_angle_state_msg.pose.orientation.y = velocity_joint_handles_[2].getPosition();
	joint_angle_state_msg.pose.orientation.z = velocity_joint_handles_[3].getPosition();
	joint_angle_state_msg.pose.position.x = velocity_joint_handles_[4].getPosition();
	joint_angle_state_msg.pose.position.y = velocity_joint_handles_[5].getPosition();
	joint_angle_state_msg.pose.position.z = velocity_joint_handles_[6].getPosition();

  pub_joint_state_.publish(joint_angle_state_msg);
}
}  // namespace franka_simple_interface
PLUGINLIB_EXPORT_CLASS(simple_franka_interface::JointVelocityInterface,controller_interface::ControllerBase)
