#pragma once

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <hexumi_msgs/msg/body_control.hpp>
#include <hexumi_msgs/msg/hexapod_motion.hpp>
#include <hexumi_msgs/msg/meta_commands.hpp>
#include <servo2040_uros/msg/current.hpp>
#include <servo2040_uros/msg/feet_contacts.hpp>
#include <servo2040_uros/msg/joint_angles.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>

#include <vector>
#include "leg.hpp"

#define DEFAULT_TIMESTEP 0.02
#define TRAJ_RES 100
#define MAX_SWING_VEL 1

class Kinematics : public rclcpp::Node {
public:
    Kinematics();

private:
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
    void set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>>);

    int num_legs;
    int num_joints;
    std::vector<std::string> leg_suffixes;
    float coxa_length, femur_length, tibia_length;
    std::vector<Leg> legs;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr      joint_states_pub_;
    sensor_msgs::msg::JointState joint_states_msg;

    rclcpp::Publisher<servo2040_uros::msg::JointAngles>::SharedPtr joint_angles_pub_;
    servo2040_uros::msg::JointAngles joint_angles_msg;

    rclcpp::Subscription<hexumi_msgs::msg::BodyControl>::SharedPtr      body_control_sub_;
    rclcpp::Subscription<hexumi_msgs::msg::HexapodMotion>::SharedPtr    hexapod_motion_sub_;
    rclcpp::Subscription<hexumi_msgs::msg::MetaCommands>::SharedPtr     meta_cmd_sub_;
    rclcpp::Subscription<servo2040_uros::msg::Current>::SharedPtr       current_sub_;
    rclcpp::Subscription<servo2040_uros::msg::FeetContacts>::SharedPtr  feet_contacts_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void meta_cmd_callback(const hexumi_msgs::msg::MetaCommands::SharedPtr);
    void hexapod_motion_callback(const hexumi_msgs::msg::HexapodMotion::SharedPtr);
    void body_control_callback(const hexumi_msgs::msg::BodyControl::SharedPtr);
    void current_callback(const servo2040_uros::msg::Current::SharedPtr);
    void feet_contacts_callback(const servo2040_uros::msg::FeetContacts::SharedPtr);
    void timer_callback();
    void publish_joint_states();

    bool is_power_on;
    bool is_ready;
    bool imu_status;

    bool is_sitting;

    KDL::Frame body_pose;
    KDL::Twist body_twist;

    KDL::Twist motion_twist; // motion velocity twist in virtual body frame
    int gait_type; // 0 for wave, 1 for ripple, 2 for tripod
    bool is_omnidirectional; // false for streamlined, true or omnidirectional  

    double total_current;
    std::vector<bool> feet_states;

    bool params_set_flag;

    double max_lin_velocity;
    double max_ang_velocity;

    double transfer_height;

    bool is_in_null_pos;

    void sit_down();
    void stand_up();

    void move_robot();
    void move_wave();
    void move_ripple();
    void move_tripod();

    void move_body();

    void center_legs();
};