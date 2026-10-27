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

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "leg.hpp"

#define DEFAULT_TIMESTEP 0.02
#define TRAJ_RES 100
#define MAX_SWING_VEL 1

class Kinematics : public rclcpp::Node {
public:
    Kinematics();

private:
    void load_params_from_global_param_server();

    int num_legs = 0;
    int num_joints = 0; // total number of joints, num_legs * joints per leg
    std::vector<std::string> leg_suffixes;
    float coxa_length = 0.0f, femur_length = 0.0f, tibia_length = 0.0f;
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

    // The gait runs blocking loops inside the timer callback. Subscriptions live in their own
    // callback group so a multi-threaded executor keeps receiving commands (e.g. power off)
    // while a step is in progress.
    rclcpp::CallbackGroup::SharedPtr command_cb_group_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_;

    void meta_cmd_callback(const hexumi_msgs::msg::MetaCommands::SharedPtr);
    void hexapod_motion_callback(const hexumi_msgs::msg::HexapodMotion::SharedPtr);
    void body_control_callback(const hexumi_msgs::msg::BodyControl::SharedPtr);
    void current_callback(const servo2040_uros::msg::Current::SharedPtr);
    void feet_contacts_callback(const servo2040_uros::msg::FeetContacts::SharedPtr);
    void timer_callback();
    void publish_joint_states();

    // written by command callbacks, polled by the control loop
    std::atomic<bool> is_power_on{false};
    std::atomic<bool> is_ready{false};
    std::atomic<bool> imu_status{false};

    // latest commands, guarded by command_mutex_ and copied into the control state once per tick
    std::mutex command_mutex_;
    KDL::Frame cmd_body_pose = KDL::Frame::Identity();
    KDL::Twist cmd_body_twist = KDL::Twist::Zero();
    KDL::Twist cmd_motion_twist = KDL::Twist::Zero();
    int cmd_gait_type = hexumi_msgs::msg::HexapodMotion::TRIPOD_GAIT;
    bool cmd_is_omnidirectional = false;
    void fetch_commands();

    // control state, only touched by the control loop
    bool is_sitting = true;

    KDL::Frame body_pose = KDL::Frame::Identity();
    KDL::Twist body_twist = KDL::Twist::Zero();

    KDL::Twist motion_twist = KDL::Twist::Zero(); // motion velocity twist in virtual body frame
    int gait_type = hexumi_msgs::msg::HexapodMotion::TRIPOD_GAIT; // 0 for wave, 1 for ripple, 2 for tripod
    bool is_omnidirectional = false; // false for streamlined, true or omnidirectional
    bool warned_unsupported_gait = false;

    std::atomic<double> total_current{0.0};

    double max_lin_velocity = 0.0;
    double max_ang_velocity = 0.0;

    double transfer_height = 0.0;

    bool is_in_null_pos = false;

    // moves every leg from its sitting position to its null position (or back) via a point above/below it
    void move_legs_vertically_first(bool standing_up);
    // samples all legs' trajectories from 0 to duration (inclusive), publishing every dt
    bool play_trajectories(const std::vector<int>& leg_ids, double duration, double dt, bool abortable);

    void sit_down();
    void stand_up();

    void move_robot();
    bool move_tripod(); // false if the step was aborted

    void move_body();

    void center_legs();
};
