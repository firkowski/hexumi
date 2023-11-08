#include "hexumi_teleop_joy.hpp"

using namespace std::chrono_literals;

TeleopJoy::TeleopJoy() 
: Node("hexumi_teleop_joy") {
    press_to_power_on_duration = 3.0;
    
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for global parameter server to be available...");
    parameters_client_->wait_for_service();

    auto parameters_future = parameters_client_->get_parameters({"max_lin_velocity", 
                                                                "max_ang_velocity",
                                                                "max_body_y_euler",
                                                                "max_body_z_euler",
                                                                "max_body_x",
                                                                "max_body_y",
                                                                "max_body_velocity_z"},
            std::bind(&TeleopJoy::set_params_from_global_param_server, this, std::placeholders::_1));

    servo_power_flag     = false;
    start_flag           = false;
    imu_flag             = false;

    startTime = std::chrono::steady_clock::now();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));

    body_control_pub_   = create_publisher<hexumi_msgs::msg::BodyControl>("/teleop/body_control", 1);           // body pose
    hexapod_motion_pub_ = create_publisher<hexumi_msgs::msg::HexapodMotion>("/teleop/hexapod_motion", 1); // low lever power, stand up, sit down stuff
    meta_cmd_pub_       = create_publisher<hexumi_msgs::msg::MetaCommands>("/teleop/meta_cmd", 1);        // twist  of {b} or direction of {b}, motion type 
}
void TeleopJoy::set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>> future) {
    // get params from global param server
    max_lin_velocity    = future.get()[0].as_double();
    max_ang_velocity    = future.get()[1].as_double();
    max_body_y_euler    = future.get()[2].as_double();
    max_body_z_euler    = future.get()[3].as_double();
    max_body_x          = future.get()[4].as_double();
    max_body_y          = future.get()[5].as_double();
    max_body_velocity_z = future.get()[6].as_double();
}

// sets control flags depending on joy input
void TeleopJoy::setJoyFlags(const sensor_msgs::msg::Joy::SharedPtr &joy){
    servo_power_off_command_flag = joy->buttons[button_H] && servo_power_flag; // press
    servo_power_on_command_flag  = joy->buttons[button_ZR] && joy->buttons[button_ZL] && !servo_power_flag; // hold
    start_command_flag           = joy->buttons[button_B]; // press
    imu_on_off_command_flag      = joy->buttons[button_O]; // press
    omnidirectional_command_flag = joy->buttons[button_R]; // hold
    wave_gait_command_flag       = joy->buttons[button_Y]; // press 
    ripple_gait_command_flag     = joy->buttons[button_X]; // press
    tripod_gait_command_flag     = joy->buttons[button_A]; // press
}

// DEFINITIONS OF CONTROL METHODS
void TeleopJoy::servoPowerOn() {
    servo_power_flag = true;
    meta_cmd.power = true;
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

void TeleopJoy::servoPowerOff() {
    servo_power_flag = false;
    meta_cmd.power = false;
    start_flag = false;
    meta_cmd.is_ready = false;
    imu_flag = false;
    meta_cmd.imu_status = false;
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

void TeleopJoy::scanForPowerOn() {
    auto currentTime = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);

    if (duration.count() >= press_to_power_on_duration) 
        servoPowerOn();
}

//stand up or sit down
void TeleopJoy:: startStop() {
    start_flag ^= 1;
    meta_cmd.is_ready ^= 1;
    if (!start_flag) {
        imu_flag = false;
        meta_cmd.imu_status =false;
    }
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

void TeleopJoy::imuOnOff() {
    imu_flag ^= 1;
    meta_cmd.imu_status ^= 1;
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

// set motion type (omni vs stream)
void TeleopJoy::setOmnidirectional() {
    hexapod_motion.motion_type = hexumi_msgs::msg::HexapodMotion::MOTION_OMNI;
    hexapod_motion_pub_->publish(hexapod_motion);
    // sleep here ???
}

void TeleopJoy::setStreamlined() {
    hexapod_motion.motion_type = hexumi_msgs::msg::HexapodMotion::MOTION_STREAM;
    hexapod_motion_pub_->publish(hexapod_motion);
    // sleep here ??
}

// set gait (wave, ripple, tripod)
void TeleopJoy::setGaitWave() {
    hexapod_motion.gait= hexumi_msgs::msg::HexapodMotion::WAVE_GAIT;
    hexapod_motion_pub_->publish(hexapod_motion);
}

void TeleopJoy::setGaitRipple() {
    hexapod_motion.gait= hexumi_msgs::msg::HexapodMotion::RIPPLE_GAIT;
    hexapod_motion_pub_->publish(hexapod_motion);
}

void TeleopJoy::setGaitTripod() {
    hexapod_motion.gait= hexumi_msgs::msg::HexapodMotion::TRIPOD_GAIT;
    hexapod_motion_pub_->publish(hexapod_motion);
}

// find hexapod motion data
// find body control
void TeleopJoy::setBodyPose(double s_xR, double s_yR, int R_button_state) {
    if (R_button_state == 1.0) {
        body_control.body_pose_euler_angles.euler_angles.z = 0.0;
        body_control.body_pose_euler_angles.euler_angles.y = 0.0;
        body_control.body_pose_euler_angles.euler_angles.x = 0.0;

        body_control.body_pose_euler_angles.position.x = -s_xR * max_body_x;
        body_control.body_pose_euler_angles.position.y = s_yR * max_body_y;
        body_control.body_pose_euler_angles.position.z = 0.0;
    }
    else {
        body_control.body_pose_euler_angles.euler_angles.z = s_yR * max_body_z_euler;
        body_control.body_pose_euler_angles.euler_angles.y = s_xR * max_body_y_euler;
        body_control.body_pose_euler_angles.euler_angles.x = 0.0;

        body_control.body_pose_euler_angles.position.x = 0.0;
        body_control.body_pose_euler_angles.position.y = 0.0;
        body_control.body_pose_euler_angles.position.z = 0.0;
    }
}

// ZYX rotation, Quaterions in JPL convention
void TeleopJoy::setBodyTwist(double z_relative) {
    body_control.body_twist.angular.x = 0.0;
    body_control.body_twist.angular.y = 0.0;
    body_control.body_twist.angular.z = 0.0;

    body_control.body_twist.linear.x  = 0.0;
    body_control.body_twist.linear.y  = 0.0;
    body_control.body_twist.linear.z = z_relative * max_body_velocity_z;
}

// find motion twist
void TeleopJoy::setStreamTwist(double s_xL, double s_yL){
    hexapod_motion.motion_twist.angular.x = 0.0;
    hexapod_motion.motion_twist.angular.y = 0.0;
    hexapod_motion.motion_twist.angular.z = -s_yL * max_ang_velocity;

    hexapod_motion.motion_twist.linear.x  = s_xL * max_lin_velocity;
    hexapod_motion.motion_twist.linear.y  = 0.0;
    hexapod_motion.motion_twist.linear.z  = 0.0;
}

void TeleopJoy::setOmniTwist(double s_xL, double s_yL) {
    hexapod_motion.motion_twist.angular.x = 0.0;
    hexapod_motion.motion_twist.angular.y = 0.0;
    hexapod_motion.motion_twist.angular.z = 0.0;

    hexapod_motion.motion_twist.linear.x  = s_xL * max_lin_velocity;
    hexapod_motion.motion_twist.linear.y  = -s_yL * max_lin_velocity;
    hexapod_motion.motion_twist.linear.z  = 0.0;
}

void TeleopJoy::sendHexapodMotionData(const sensor_msgs::msg::Joy::SharedPtr joy) {

    setBodyPose(joy->axes[axis_right_x],
                joy->axes[axis_right_y],
                joy->buttons[button_L]);

    setBodyTwist(joy->axes[axis_cross_x]);

    omnidirectional_command_flag ?   setOmniTwist   (joy->axes[axis_left_x], joy->axes[axis_left_y]) 
                                   : setStreamTwist (joy->axes[axis_left_x], joy->axes[axis_left_y]);

    hexapod_motion_pub_->publish(hexapod_motion);
    body_control_pub_  ->publish(body_control);
}

// CALLBACK
void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    setJoyFlags(joy);
    // H pressed
    if (servo_power_off_command_flag)
        servoPowerOff();

    // ZL and ZR pressed
    if (servo_power_on_command_flag)
        scanForPowerOn();
    else 
        startTime = std::chrono::steady_clock::now();

    if (servo_power_flag) {
        // O pressed

        if (start_command_flag)
            startStop();

        if (start_flag) {

            if (imu_on_off_command_flag)
                imuOnOff();

            if (omnidirectional_command_flag)
                setOmnidirectional();
            else
                setStreamlined();

            if (wave_gait_command_flag)
                setGaitWave();
            else if (ripple_gait_command_flag)
                setGaitRipple();
            else if (tripod_gait_command_flag)
                setGaitTripod();

            sendHexapodMotionData(joy);
        }
    }
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Pro Controller teleop converter, take care of your controller now.");
	
	rclcpp::spin(std::make_shared<TeleopJoy>());
    rclcpp::shutdown();
    return 0;
}