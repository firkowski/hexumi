#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <hexumi_msgs/msg/body_control.hpp>
#include <hexumi_msgs/msg/hexapod_motion.hpp>
#include <hexumi_msgs/msg/meta_commands.hpp>

#include <Eigen/Geometry>
#include <cmath>

#define PRO_CONTROLLER_AXIS_STICK_LEFT_HORIZONTAL   0
#define PRO_CONTROLLER_AXIS_STICK_LEFT_VERTICAL     1
#define PRO_CONTROLLER_AXIS_STICK_RIGHT_HORIZONTAL  2
#define PRO_CONTROLLER_AXIS_STICK_RIGHT_VERTICAL    3
#define PRO_CONTROLLER_AXIS_CROSS_HORIZONTAL        4
#define PRO_CONTROLLER_AXIS_CROSS_VERTICAL          5

#define PRO_CONTROLLER_BUTTON_ACTION_B              0
#define PRO_CONTROLLER_BUTTON_ACTION_A              1                
#define PRO_CONTROLLER_BUTTON_ACTION_X              2
#define PRO_CONTROLLER_BUTTON_ACTION_Y              3
#define PRO_CONTROLLER_BUTTON_O                     4
#define PRO_CONTROLLER_BUTTON_SHOULDER_LEFT         5
#define PRO_CONTROLLER_BUTTON_SHOULDER_RIGHT        6
#define PRO_CONTROLLER_BUTTON_TRIGGER_LEFT          7
#define PRO_CONTROLLER_BUTTON_TRIGGER_RIGHT         8
#define PRO_CONTROLLER_BUTTON_MINUS                 9
#define PRO_CONTROLLER_BUTTON_PLUS                 10
#define PRO_CONTROLLER_BUTTON_H                    11
#define PRO_CONTROLLER_BUTTON_STICK_LEFT           12
#define PRO_CONTROLLER_BUTTON_STICK_RIGHT          13

class TeleopJoy : public rclcpp::Node {
public:
    TeleopJoy();

private:

    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
    void set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>>);

    rclcpp::Publisher<hexumi_msgs::msg::BodyControl>::SharedPtr   body_control_pub_;
    rclcpp::Publisher<hexumi_msgs::msg::HexapodMotion>::SharedPtr hexapod_motion_pub_;
    rclcpp::Publisher<hexumi_msgs::msg::MetaCommands>::SharedPtr  meta_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    hexumi_msgs::msg::BodyControl   body_control;
    hexumi_msgs::msg::HexapodMotion hexapod_motion;
    hexumi_msgs::msg::MetaCommands  meta_cmd;
 
    bool servo_power_flag;
    bool start_flag;
    bool imu_flag;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr);

    // control methods
    void servoPowerOn();
    void servoPowerOff();
    void scanForPowerOn();
    void startStop();
    void imuOnOff();
    void setOmnidirectional();
    void setStreamlined();
    void setGaitWave();
    void setGaitRipple();
    void setGaitTripod();

    void sendHexapodMotionData(const sensor_msgs::msg::Joy::SharedPtr);

    void setBodyPose(double, double, int);
    void setBodyTwist(double);

    void setStreamTwist(double, double);
    void setOmniTwist(double, double);

    // joy control flags
    bool servo_power_off_command_flag;
    bool servo_power_on_command_flag;
    bool start_command_flag;
    bool imu_on_off_command_flag;
    bool omnidirectional_command_flag;
    bool wave_gait_command_flag;
    bool ripple_gait_command_flag;
    bool tripod_gait_command_flag;

    // set joy control flags
    void setJoyFlags(const sensor_msgs::msg::Joy::SharedPtr&);

    std::chrono::time_point<std::chrono::steady_clock> startTime;

    double max_lin_velocity;
    double max_ang_velocity;
    double max_body_y_euler;
    double max_body_z_euler;
    double max_body_x;
    double max_body_y;
    double max_body_velocity_z;

    static constexpr int axis_left_y        = PRO_CONTROLLER_AXIS_STICK_LEFT_HORIZONTAL;
    static constexpr int axis_left_x        = PRO_CONTROLLER_AXIS_STICK_LEFT_VERTICAL;
    static constexpr int axis_right_y       = PRO_CONTROLLER_AXIS_STICK_RIGHT_HORIZONTAL;
    static constexpr int axis_right_x       = PRO_CONTROLLER_AXIS_STICK_RIGHT_VERTICAL;
    static constexpr int axis_cross_y       = PRO_CONTROLLER_AXIS_CROSS_HORIZONTAL;
    static constexpr int axis_cross_x       = PRO_CONTROLLER_AXIS_CROSS_VERTICAL;

    static constexpr int button_B           = PRO_CONTROLLER_BUTTON_ACTION_B;
    static constexpr int button_A           = PRO_CONTROLLER_BUTTON_ACTION_A;
    static constexpr int button_X           = PRO_CONTROLLER_BUTTON_ACTION_X;
    static constexpr int button_Y           = PRO_CONTROLLER_BUTTON_ACTION_Y;
    static constexpr int button_O           = PRO_CONTROLLER_BUTTON_O;
    static constexpr int button_L           = PRO_CONTROLLER_BUTTON_SHOULDER_LEFT;
    static constexpr int button_R           = PRO_CONTROLLER_BUTTON_SHOULDER_RIGHT;
    static constexpr int button_ZL          = PRO_CONTROLLER_BUTTON_TRIGGER_LEFT;
    static constexpr int button_ZR          = PRO_CONTROLLER_BUTTON_TRIGGER_RIGHT;
    static constexpr int button_MINUS       = PRO_CONTROLLER_BUTTON_MINUS;
    static constexpr int button_PLUS        = PRO_CONTROLLER_BUTTON_PLUS;
    static constexpr int button_H           = PRO_CONTROLLER_BUTTON_H;
    static constexpr int button_stick_L     = PRO_CONTROLLER_BUTTON_STICK_LEFT;
    static constexpr int button_stick_R     = PRO_CONTROLLER_BUTTON_STICK_RIGHT;

    double press_to_power_on_duration;
};