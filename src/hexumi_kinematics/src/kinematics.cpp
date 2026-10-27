#include "kinematics.hpp"

#include <algorithm>
#include <map>

Kinematics::Kinematics() 
: Node ("hexumi_kinematics") {
    load_params_from_global_param_server();

    command_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_cb_group_;

    meta_cmd_sub_       = this->create_subscription<hexumi_msgs::msg::MetaCommands>
        ("/teleop/meta_cmd", 1, std::bind(&Kinematics::meta_cmd_callback, this, std::placeholders::_1), command_options);

    hexapod_motion_sub_ = this->create_subscription<hexumi_msgs::msg::HexapodMotion>
        ("/teleop/hexapod_motion", 1, std::bind(&Kinematics::hexapod_motion_callback, this, std::placeholders::_1), command_options);

    body_control_sub_   = this->create_subscription<hexumi_msgs::msg::BodyControl>
        ("/teleop/body_control", 1, std::bind(&Kinematics::body_control_callback, this, std::placeholders::_1), command_options);

    //current_sub_        = this->create_subscription<servo2040_uros::msg::Current>
    //    ("/servo2040/current", 1, std::bind(&Kinematics::current_callback, this, std::placeholders::_1), command_options);

    //feet_contacts_sub_  = this->create_subscription<servo2040_uros::msg::FeetContacts>
    //    ("/servo2040/feet_contacts", 1, std::bind(&Kinematics::feet_contacts_callback, this, std::placeholders::_1), command_options);

    joint_states_pub_   = this->create_publisher<sensor_msgs::msg::JointState>("/kinematics/joint_states", 1);

    joint_angles_pub_   = this->create_publisher<servo2040_uros::msg::JointAngles>("/joint_angles", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&Kinematics::timer_callback, this), control_cb_group_);
}

void Kinematics::load_params_from_global_param_server() {
    auto client_node = std::make_shared<rclcpp::Node>(std::string(this->get_name()) + "_param_client");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(client_node, "/global_parameter_server");

    RCLCPP_INFO(this->get_logger(), "Waiting for global parameter server to be available...");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok())
            throw std::runtime_error("interrupted while waiting for the global parameter server");
    }

    auto get = [&parameters_client](const std::vector<std::string>& names) {
        std::map<std::string, rclcpp::Parameter> params;
        for (auto& param : parameters_client->get_parameters(names)) {
            params.emplace(param.get_name(), param);
        }
        for (auto& name : names) {
            auto it = params.find(name);
            if (it == params.end() || it->second.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
                throw std::runtime_error("global parameter server does not provide '" + name + "'");
        }
        return params;
    };

    auto p = get({"num_legs", "num_leg_joints", "leg_suffixes",
                  "coxa_length", "femur_length", "tibia_length",
                  "leg_sitting_z",
                  "workspace_ellipse_origin", "workspace_ellipse_axes",
                  "max_lin_velocity", "max_ang_velocity",
                  "transfer_height"});

    num_legs     = p.at("num_legs").as_int();
    num_joints   = p.at("num_leg_joints").as_int() * num_legs;
    leg_suffixes = p.at("leg_suffixes").as_string_array();

    if (static_cast<int>(leg_suffixes.size()) != num_legs)
        throw std::runtime_error("leg_suffixes has " + std::to_string(leg_suffixes.size()) +
                                 " entries but num_legs is " + std::to_string(num_legs));
    if (static_cast<size_t>(num_joints) != joint_angles_msg.joint_angles.size())
        throw std::runtime_error("servo2040_uros/JointAngles holds " + std::to_string(joint_angles_msg.joint_angles.size()) +
                                 " angles but the robot has " + std::to_string(num_joints) + " joints");

    coxa_length  = p.at("coxa_length").as_double();
    femur_length = p.at("femur_length").as_double();
    tibia_length = p.at("tibia_length").as_double();

    std::vector<double> ws_origin = p.at("workspace_ellipse_origin").as_double_array();
    std::vector<double> ws_axes   = p.at("workspace_ellipse_axes").as_double_array();
    if (ws_origin.size() != 3 || ws_axes.size() != 3)
        throw std::runtime_error("workspace_ellipse_origin and workspace_ellipse_axes must have 3 entries");
    EllipsoidWorkspace::setCenterL(KDL::Vector(ws_origin[0], ws_origin[1], ws_origin[2]));
    EllipsoidWorkspace::setAxes(ws_axes);

    double sitting_z = p.at("leg_sitting_z").as_double();

    max_lin_velocity = p.at("max_lin_velocity").as_double();
    max_ang_velocity = p.at("max_ang_velocity").as_double();

    transfer_height = p.at("transfer_height").as_double();

    std::vector<std::string> leg_param_names;
    for (auto& suffix : leg_suffixes) {
        leg_param_names.push_back(suffix + "_origin");
        leg_param_names.push_back(suffix + "_yaw");
        leg_param_names.push_back(suffix + "_sitting_radius");
    }
    auto leg_params = get(leg_param_names);

    // initialize legs from global parameters
    legs.reserve(num_legs);
    for (int i = 0; i < num_legs; i++) {
        const std::string& suffix = leg_suffixes[i];
        std::vector<double> origin_xyz = leg_params.at(suffix + "_origin").as_double_array();
        if (origin_xyz.size() != 3)
            throw std::runtime_error(suffix + "_origin must have 3 entries");
        KDL::Vector origin = KDL::Vector(origin_xyz[0], origin_xyz[1], origin_xyz[2]);

        double yaw = leg_params.at(suffix + "_yaw").as_double();
        KDL::Frame T_origin = KDL::Frame(KDL::Rotation::RPY(0,0,yaw), origin);

        double sitting_radius = leg_params.at(suffix + "_sitting_radius").as_double();
        
        KDL::Vector sitting_position = KDL::Vector(sitting_radius * std::cos(yaw), 
                                                   sitting_radius * std::sin(yaw), 
                                                   sitting_z);
        legs.emplace_back(i, 
                          coxa_length, femur_length, tibia_length, 
                          T_origin, 
                          sitting_position);
        
        joint_states_msg.name.push_back("coxa_joint_" + suffix);
        joint_states_msg.name.push_back("femur_joint_" + suffix);
        joint_states_msg.name.push_back("tibia_joint_" + suffix);
        joint_states_msg.position.push_back(0.0);
        joint_states_msg.position.push_back(0.0);
        joint_states_msg.position.push_back(0.0);
    }
    joint_angles_msg.joint_angles.fill(0.0f);

    RCLCPP_INFO(this->get_logger(), "Loaded parameters for %d legs.", num_legs);
}

void Kinematics::meta_cmd_callback(const hexumi_msgs::msg::MetaCommands::SharedPtr meta_cmd_msg) {
    this->is_power_on = meta_cmd_msg->power;
    this->is_ready    = meta_cmd_msg->is_ready;
    this->imu_status  = meta_cmd_msg->imu_status;
}

void Kinematics::hexapod_motion_callback(const hexumi_msgs::msg::HexapodMotion::SharedPtr hexapod_motion_msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    this->cmd_gait_type          = hexapod_motion_msg->gait;
    this->cmd_is_omnidirectional = hexapod_motion_msg->motion_type;
    this->cmd_motion_twist = KDL::Twist(KDL::Vector(hexapod_motion_msg->motion_twist.linear.x,
                                                    hexapod_motion_msg->motion_twist.linear.y,
                                                    hexapod_motion_msg->motion_twist.linear.z),

                                        KDL::Vector(hexapod_motion_msg->motion_twist.angular.x,
                                                    hexapod_motion_msg->motion_twist.angular.y,
                                                    hexapod_motion_msg->motion_twist.angular.z));
}

void Kinematics::body_control_callback(const hexumi_msgs::msg::BodyControl::SharedPtr body_control_msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    this->cmd_body_twist = KDL::Twist(KDL::Vector(body_control_msg->body_twist.linear.x,
                                                  body_control_msg->body_twist.linear.y,
                                                  body_control_msg->body_twist.linear.z),

                                      KDL::Vector(body_control_msg->body_twist.angular.x,
                                                  body_control_msg->body_twist.angular.y,
                                                  body_control_msg->body_twist.angular.z));
    
    double body_z_angle = body_control_msg->body_pose_euler_angles.euler_angles.z;
    double body_y_angle = body_control_msg->body_pose_euler_angles.euler_angles.y;
    double body_x_angle = body_control_msg->body_pose_euler_angles.euler_angles.x;

    double body_x       = body_control_msg->body_pose_euler_angles.position.x;
    double body_y       = body_control_msg->body_pose_euler_angles.position.y;
    double body_z       = body_control_msg->body_pose_euler_angles.position.z;

    KDL::Rotation body_rotation = KDL::Rotation::EulerZYX(body_z_angle, body_y_angle, body_x_angle);
    KDL::Vector body_position = KDL::Vector(body_x, body_y, body_z);

    this->cmd_body_pose = KDL::Frame(body_rotation, body_position);
}

void Kinematics::current_callback(const servo2040_uros::msg::Current::SharedPtr current_msg) {
    this->total_current = current_msg->current;
}

void Kinematics::feet_contacts_callback(const servo2040_uros::msg::FeetContacts::SharedPtr feet_contacts_msg) {
    int n = std::min(num_legs, static_cast<int>(feet_contacts_msg->is_active.size()));
    for (int i = 0; i < n; i++) {
        feet_contacts_msg->is_active[i] ? legs[i].activate() : legs[i].deactivate();
    }
}

void Kinematics::fetch_commands() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    body_pose          = cmd_body_pose;
    body_twist         = cmd_body_twist;
    motion_twist       = cmd_motion_twist;
    gait_type          = cmd_gait_type;
    is_omnidirectional = cmd_is_omnidirectional;
    Leg::motion_twist  = motion_twist;
}

void Kinematics::timer_callback() {
    fetch_commands();
    publish_joint_states();
    if(!is_power_on) { 
        if (!is_sitting)
            sit_down();
        return;
    }
    if(is_ready) {
        if(is_sitting)
            stand_up();

        if(!KDL::Equal(motion_twist, KDL::Twist::Zero(), KDL::epsilon * 5e4))
            move_robot();
        else {
            if (!is_in_null_pos)
                center_legs();
        
            move_body();
        }
        
    }
    else {
        if(!is_sitting) {
            sit_down();
        }
    }
}

void Kinematics::publish_joint_states() {
    joint_states_msg.header.stamp = this->now();
    for (int i = 0; i < num_legs; i++) {
        joint_states_msg.position[3*i]   = legs[i].getCoxaAngle();
        joint_states_msg.position[3*i+1] = legs[i].getFemurAngle();
        joint_states_msg.position[3*i+2] = legs[i].getTibiaAngle();
    } 
    joint_states_pub_->publish(joint_states_msg);

    for (int i = 0; i < num_legs; i++) {
        joint_angles_msg.joint_angles[3*i]   = legs[i].getCoxaAngle();
        joint_angles_msg.joint_angles[3*i+1] = legs[i].getFemurAngle();
        joint_angles_msg.joint_angles[3*i+2] = legs[i].getTibiaAngle();
    } 
    joint_angles_pub_->publish(joint_angles_msg);
    for (Leg& leg : legs) {
        leg.resetDuration();
    }
}

bool Kinematics::play_trajectories(const std::vector<int>& leg_ids, double duration, double dt, bool abortable) {
    const auto dt_ns = std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9));
    for (double t = 0.0; ; t += dt) {
        if (!rclcpp::ok() || (abortable && (!is_ready || !is_power_on)))
            return false;
        double t_clamped = std::min(t, duration);
        for (int i : leg_ids) {
            legs[i].setPositionB(legs[i].traj->Pos(t_clamped).p);
        }
        publish_joint_states();
        if (t_clamped >= duration)
            return true;
        rclcpp::sleep_for(dt_ns);
    }
}

// Gait
void Kinematics::move_legs_vertically_first(bool standing_up) {
    std::vector<int> all_legs;
    double duration = 0.0;
    for (int i = 0; i < num_legs; i++) {
        KDL::Vector sitting = legs[i].getSittingPositionB();
        KDL::Vector null    = legs[i].getNullPositionB();
        KDL::Vector start   = standing_up ? sitting : legs[i].getCurrentPositionB();
        KDL::Vector end     = standing_up ? null    : sitting;
        KDL::Vector via     = standing_up ? KDL::Vector(null.x(), null.y(), sitting.z())
                                          : KDL::Vector(start.x(), start.y(), sitting.z());

        auto path = new KDL::Path_RoundedComposite(standing_up ? 0.003 : 0.0003, 0.01, new KDL::RotationalInterpolation_SingleAxis());
        path->Add(KDL::Frame(KDL::Rotation::Identity(), start));
        if (!KDL::Equal(via, start, 1e-6) && !KDL::Equal(via, end, 1e-6))
            path->Add(KDL::Frame(KDL::Rotation::Identity(), via));
        path->Add(KDL::Frame(KDL::Rotation::Identity(), end));
        path->Finish();
        legs[i].setSegmentTrajectory(path, new KDL::VelocityProfile_Trap(0.5, 0.1));

        duration = std::max(duration, legs[i].traj->Duration());
        all_legs.push_back(i);
    }
    play_trajectories(all_legs, duration, DEFAULT_TIMESTEP, false);
}

void Kinematics::sit_down() {
    move_legs_vertically_first(false);
    is_sitting = true;
    is_in_null_pos = false;
}

void Kinematics::stand_up() {
    move_legs_vertically_first(true);
    is_sitting = false;
    is_in_null_pos = true;
}

void Kinematics::move_robot() {
    publish_joint_states();
    switch (gait_type) {
        case hexumi_msgs::msg::HexapodMotion::WAVE_GAIT:
        case hexumi_msgs::msg::HexapodMotion::RIPPLE_GAIT:
            if (!warned_unsupported_gait) {
                RCLCPP_WARN(this->get_logger(), "Gait %d is not implemented, falling back to the tripod gait.", gait_type);
                warned_unsupported_gait = true;
            }
            [[fallthrough]];
        case hexumi_msgs::msg::HexapodMotion::TRIPOD_GAIT:
            if (!move_tripod())
                break;
            Leg::wavePhase = 0;
            Leg::ripplePhase = (Leg::ripplePhase + 1) % 6;
            Leg::tripodPhase = (Leg::tripodPhase + 1) % 2;
            break;
        default:
            break;
    }
}

bool Kinematics::move_tripod() {
    int phase = Leg::tripodPhase;
    std::vector<int> all_legs, transfer_legs;
    double cycle_duration = MAX_DURATION;

    this->body_pose = KDL::Frame::Identity();
    move_body();

    for (int i = 0; i < num_legs; i++) {
        all_legs.push_back(i);
        if (i % 2 == phase) {
            legs[i].generateSupportPath();
            cycle_duration = std::min(cycle_duration, legs[i].traj->Duration());
        }
        else {
            transfer_legs.push_back(i);
        }
    } 

    double speed_factor = std::sqrt(pow( (motion_twist.vel.Norm()) / (max_lin_velocity), 2) + 
                                    pow( (motion_twist.rot.Norm()) / (max_ang_velocity), 2 ));

    RCLCPP_DEBUG(this->get_logger(), "speed_factor: %f", speed_factor);

    cycle_duration *= speed_factor;
    if (cycle_duration <= KDL::epsilon) 
        return true;

    bool not_enough_swing_time = false;
    for (int i : transfer_legs) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
        if (legs[i].velprof != nullptr && legs[i].velprof->Vel(0.5*cycle_duration) > MAX_SWING_VEL) {
            not_enough_swing_time = true;
        }
    }

    if (not_enough_swing_time) {
        center_legs();
        for (int i : transfer_legs) {
            legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
        }
    }

    if (!play_trajectories(all_legs, cycle_duration, cycle_duration / TRAJ_RES, true))
        return false;

    is_in_null_pos = false;
    return true;
}

void Kinematics::move_body() {

    for (auto& leg : legs) {
        if(!leg.isInTaskspaceB(this->body_pose * Leg::body_pose.Inverse() * leg.getCurrentPositionB()))
            return;
    }
    
    for (auto& leg : legs) {
        leg.setPositionB(this->body_pose * Leg::body_pose.Inverse() * leg.getCurrentPositionB());
    } 
    Leg::body_pose = this->body_pose;
    publish_joint_states();
}

void Kinematics::center_legs() {
    this->body_pose = KDL::Frame::Identity();
    move_body();
    int phase = Leg::tripodPhase;
    double duration = 1.0;

    std::vector<int> transfer_legs;
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * 0.5, duration);
        transfer_legs.push_back(i);
    }
    play_trajectories(transfer_legs, duration, DEFAULT_TIMESTEP, false);
    is_in_null_pos = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    KDL::epsilon = 1e-9;
    try {
        auto node = std::make_shared<Kinematics>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("hexumi_kinematics"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
