#pragma once
#include <array>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <franka_hw/franka_state_interface.h>

namespace simple_franka_interface{

class JointVelocityInterface : public controller_interface::MultiInterfaceController<
                                          hardware_interface::VelocityJointInterface,
                                          franka_hw::FrankaStateInterface>{
  public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

    std::array<double, 7> initial_pose_{};

    ros::Subscriber sub_joint_velocity_target_;
    ros::Subscriber sub_joint_angle_target_;
    ros::Publisher pub_joint_state_;
    ros::Publisher pub_end_state_;

    void jointVelCallback(const geometry_msgs::PoseStamped& msg);
    void jointAngCallback(const geometry_msgs::PoseStamped& msg);
    void statePublish();
    
    std::array<double, 7> joint_velocity_target_;
    std::array<double, 7> joint_angle_target_;

    Eigen::Quaterniond q_state_;
    Eigen::Vector3d p_state_;
    double filter_params_{0.2};
    double min_step_{0.0005};

};

} 
