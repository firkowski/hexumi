#include "kinematics.hpp"

Kinematics::Kinematics() 
: Node ("hexumi_kinematics") {
    params_set_flag = false;

    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for global parameter server to be available...");
    parameters_client_->wait_for_service();

    auto parameters_future = parameters_client_->get_parameters({"num_legs","num_leg_joints",
                                                                "leg_suffixes",
                                                                
                                                                "coxa_length", "femur_length", "tibia_length",

                                                                "R1_origin", "R2_origin", "R3_origin", "L3_origin", "L2_origin", "L1_origin",
                                                                "R1_yaw"   , "R2_yaw"   , "R3_yaw"   , "L3_yaw"   , "L2_yaw"   , "L1_yaw"   ,
                                                                
                                                                "R1_sitting_radius", 
                                                                "R2_sitting_radius",
                                                                "R3_sitting_radius",
                                                                "L3_sitting_radius",
                                                                "L2_sitting_radius",
                                                                "L1_sitting_radius",
                                                                "leg_sitting_z"    ,

                                                                "workspace_ellipse_origin", "workspace_ellipse_axes",
                                                                "max_lin_velocity", "max_ang_velocity",
                                                                "transfer_height"},

            std::bind(&Kinematics::set_params_from_global_param_server, this, std::placeholders::_1));
    
    meta_cmd_sub_       = this->create_subscription<hexumi_msgs::msg::MetaCommands>
        ("/teleop/meta_cmd", 1, std::bind(&Kinematics::meta_cmd_callback, this, std::placeholders::_1));

    hexapod_motion_sub_ = this->create_subscription<hexumi_msgs::msg::HexapodMotion>
        ("/teleop/hexapod_motion", 1, std::bind(&Kinematics::hexapod_motion_callback, this, std::placeholders::_1));

    body_control_sub_   = this->create_subscription<hexumi_msgs::msg::BodyControl>
        ("/teleop/body_control", 1, std::bind(&Kinematics::body_control_callback, this, std::placeholders::_1));

    //current_sub_        = this->create_subscription<servo2040_uros::msg::Current>
    //    ("/servo2040/current", 1, std::bind(&Kinematics::current_callback, this, std::placeholders::_1));

    //feet_contacts_sub_  = this->create_subscription<servo2040_uros::msg::FeetContacts>
    //    ("/servo2040/feet_contacts", 1, std::bind(&Kinematics::feet_contacts_callback, this, std::placeholders::_1));

    joint_states_pub_   = this->create_publisher<sensor_msgs::msg::JointState>("/kinematics/joint_states", 1);

    joint_angles_pub_   = this->create_publisher<servo2040_uros::msg::JointAngles>("joint_angles", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&Kinematics::timer_callback, this));
}

void Kinematics::set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>> future) {
    num_legs     = future.get()[0].as_int();
    num_joints   = future.get()[1].as_int() * num_legs;
    leg_suffixes = future.get()[2].as_string_array();

    coxa_length  = future.get()[3].as_double();
    femur_length = future.get()[4].as_double();
    tibia_length = future.get()[5].as_double();

    std::vector<double> ws_origin = future.get()[25].as_double_array();
    EllipsoidWorkspace::setCenterL(KDL::Vector(ws_origin[0], ws_origin[1], ws_origin[2]));
    
    EllipsoidWorkspace::setAxes(future.get()[26].as_double_array());

    // initialize legs from global parameters
    double sitting_z  = future.get()[24].as_double();

    max_lin_velocity = future.get()[27].as_double();
    max_ang_velocity = future.get()[28].as_double();

    transfer_height = future.get()[29].as_double();

    legs.reserve(num_legs);
    for (int i = 0; i < num_legs; i++) {
        KDL::Vector origin = KDL::Vector(future.get()[6+i].as_double_array()[0], 
                                         future.get()[6+i].as_double_array()[1], 
                                         future.get()[6+i].as_double_array()[2]);

        double yaw = future.get()[12+i].as_double();
        KDL::Frame T_origin = KDL::Frame(KDL::Rotation::RPY(0,0,yaw), origin);

        double sitting_radius = future.get()[18+i].as_double();
        
        KDL::Vector sitting_position = KDL::Vector(sitting_radius * std::cos(yaw), 
                                                   sitting_radius * std::sin(yaw), 
                                                   sitting_z);
        RCLCPP_INFO(this->get_logger(), "leg gen");
        legs.push_back(Leg(i, 
                        coxa_length, femur_length, tibia_length, 
                        T_origin, 
                        sitting_position));
        
        RCLCPP_INFO(this->get_logger(), "msg gen");
        joint_states_msg.name.push_back("coxa_joint_" + leg_suffixes[i]);
        joint_states_msg.name.push_back("femur_joint_" + leg_suffixes[i]);
        joint_states_msg.name.push_back("tibia_joint_" + leg_suffixes[i]);
        joint_states_msg.position.push_back(0.0);
        joint_states_msg.position.push_back(0.0);
        joint_states_msg.position.push_back(0.0);
    }
    for (int i = 0; i < num_joints*num_legs; i++) {
        joint_angles_msg.joint_angles[i] = 0.0;
    }

    this->motion_twist = KDL::Twist::Zero();
    this->body_pose = KDL::Frame::Identity();

    is_sitting = true;
    params_set_flag = true;
}

void Kinematics::meta_cmd_callback(const hexumi_msgs::msg::MetaCommands::SharedPtr meta_cmd_msg) {
    this->is_power_on = meta_cmd_msg->power;
    this->is_ready    = meta_cmd_msg->is_ready;
    this->imu_status  = meta_cmd_msg->imu_status;
}

void Kinematics::hexapod_motion_callback(const hexumi_msgs::msg::HexapodMotion::SharedPtr hexapod_motion_msg) {
    this->gait_type          = hexapod_motion_msg->gait;
    this->is_omnidirectional = hexapod_motion_msg->motion_type;
    this->motion_twist = KDL::Twist(KDL::Vector(hexapod_motion_msg->motion_twist.linear.x,
                                                hexapod_motion_msg->motion_twist.linear.y,
                                                hexapod_motion_msg->motion_twist.linear.z),

                                    KDL::Vector(hexapod_motion_msg->motion_twist.angular.x,
                                                hexapod_motion_msg->motion_twist.angular.y,
                                                hexapod_motion_msg->motion_twist.angular.z));
    Leg::motion_twist = this->motion_twist;
}

void Kinematics::body_control_callback(const hexumi_msgs::msg::BodyControl::SharedPtr body_control_msg) {
    this->body_twist = KDL::Twist(KDL::Vector(body_control_msg->body_twist.linear.x,
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

    this->body_pose  = KDL::Frame(body_rotation, body_position);
}

void Kinematics::current_callback(const servo2040_uros::msg::Current::SharedPtr current_msg) {
    this->total_current = current_msg->current;
}

void Kinematics::feet_contacts_callback(const servo2040_uros::msg::FeetContacts::SharedPtr feet_contacts_msg) {
    for (int i = 0; i < num_joints; i++) {
        feet_contacts_msg->is_active[i] ? legs[i].activate() : legs[i].deactivate();
    }
}

void Kinematics::timer_callback() {
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
    if (params_set_flag) {
        for (int i = 0; i < num_legs; i++) {
            joint_states_msg.header.stamp.sec = this->now().seconds();
            joint_states_msg.header.stamp.nanosec = this->now().nanoseconds();
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
}

// Gait
void Kinematics::sit_down() {
    double dt = DEFAULT_TIMESTEP;

    std::vector<KDL::Path_RoundedComposite*> paths;
    for (int i = 0; i < num_legs; i++) {
        paths.push_back(new KDL::Path_RoundedComposite(0.0003, 0.01, new KDL::RotationalInterpolation_SingleAxis()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getNullPositionB()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(legs[i].getNullPositionB().x(), 
                                                                        legs[i].getNullPositionB().y(),
                                                                        legs[i].getSittingPositionB().z())));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getSittingPositionB()));
        paths[i]->Finish();
    }
    std::vector<KDL::VelocityProfile*> velprofs;
    for (int i = 0; i < num_legs; i++) {
        velprofs.push_back(new KDL::VelocityProfile_Trap(0.5,0.1));
        velprofs[i]->SetProfile(0,paths[i]->PathLength());  
    }

    std::vector<KDL::Trajectory*> trajs;
    for (int i = 0; i < num_legs; i++) {
        trajs.push_back(new KDL::Trajectory_Segment(paths[i], velprofs[i]));
    }

	std::vector<KDL::Trajectory_Composite*> ctrajs;
    for (int i = 0; i < num_legs; i++) {
        ctrajs.push_back(new KDL::Trajectory_Composite());
        ctrajs[i]->Add(trajs[i]);
    }

    for (double t=0.0; t<=trajs[0]->Duration(); t+=dt) {
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB((trajs[i]->Pos(t)).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    for (auto ctraj : ctrajs) {delete ctraj;}

    is_sitting = true;
    is_in_null_pos = false;
}

void Kinematics::stand_up() {
    std::vector<KDL::Path_RoundedComposite*> paths;
    for (int i = 0; i < num_legs; i++) {
        paths.push_back(new KDL::Path_RoundedComposite(0.003, 0.01, new KDL::RotationalInterpolation_SingleAxis()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getSittingPositionB()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(legs[i].getNullPositionB().x(), 
                                                                        legs[i].getNullPositionB().y(),
                                                                        legs[i].getSittingPositionB().z())));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getNullPositionB()));
        paths[i]->Finish();
    }
    std::vector<KDL::VelocityProfile*> velprofs;
    for (int i = 0; i < num_legs; i++) {
        velprofs.push_back(new KDL::VelocityProfile_Trap(0.5,0.1));
        velprofs[i]->SetProfile(0,paths[i]->PathLength());  
    }

    std::vector<KDL::Trajectory*> trajs;
    for (int i = 0; i < num_legs; i++) {
        trajs.push_back(new KDL::Trajectory_Segment(paths[i], velprofs[i]));
    }

	std::vector<KDL::Trajectory_Composite*> ctrajs;
    for (int i = 0; i < num_legs; i++) {
        ctrajs.push_back(new KDL::Trajectory_Composite());
        ctrajs[i]->Add(trajs[i]);
    }

    double dt = 0.02;
    for (double t=0.0; t<=trajs[0]->Duration(); t+=dt) {
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB((trajs[i]->Pos(t)).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    for (auto ctraj : ctrajs) {delete ctraj;}

    is_sitting = false;
    is_in_null_pos = true;
}

void Kinematics::move_robot() {
    publish_joint_states();
    switch (gait_type) {
        case 0:
            //move_wave();
            break;
        case 1:
            //move_ripple();
            break;
        case 2:
            move_tripod();
            Leg::wavePhase = 0;
            Leg::ripplePhase = (Leg::ripplePhase + 1) % 6;
            Leg::tripodPhase = (Leg::tripodPhase + 1) % 2;
            break;
        default:
            break;
    }
}

void Kinematics::move_tripod() {
    int phase = Leg::tripodPhase;

    this->body_pose = KDL::Frame::Identity();
    move_body();

    double cycle_duration = MAX_DURATION;
    for (int i = phase; i < num_legs; i+=2) {
        legs[i].generateSupportPath();
        cycle_duration = std::min(cycle_duration, legs[i].traj->Duration());
    } 

    double speed_factor = std::sqrt(pow( (motion_twist.vel.Norm()) / (max_lin_velocity), 2) + 
                                    pow( (motion_twist.rot.Norm()) / (max_ang_velocity), 2 ));

    RCLCPP_INFO(this->get_logger(), "speed_factor: %f", speed_factor);

    cycle_duration *= speed_factor;
    if (cycle_duration <= KDL::epsilon) 
        return;

    bool not_enough_swing_time = false;
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
        if (legs[i].velprof->Vel(0.5*cycle_duration) > MAX_SWING_VEL) {
            not_enough_swing_time = true;
        }
    }

    if (not_enough_swing_time) center_legs();
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
    }
    

    double dt = cycle_duration / TRAJ_RES;
    for (double t = 0.0; std::abs(t) < cycle_duration; t+=dt) {
        if(!is_ready || !is_power_on) return;
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB(legs[i].traj->Pos(t).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    is_in_null_pos = false;
}

void Kinematics::move_wave() {
    int phase = Leg::tripodPhase;

    this->body_pose = KDL::Frame::Identity();
    move_body();

    double cycle_duration = MAX_DURATION;
    for (int i = phase; i < num_legs; i+=6) {
        legs[i].generateSupportPath();
        cycle_duration = std::min(cycle_duration, legs[i].traj->Duration());
    } 

    double speed_factor = std::sqrt(pow( (motion_twist.vel.Norm()) / (max_lin_velocity), 2) + 
                                    pow( (motion_twist.rot.Norm()) / (max_ang_velocity), 2 ));

    RCLCPP_INFO(this->get_logger(), "speed_factor: %f", speed_factor);

    cycle_duration *= speed_factor / 6;
    if (cycle_duration <= KDL::epsilon) 
        return;

    bool not_enough_swing_time = false;
    for (int i = (phase+1)%6; i < num_legs; i+=6) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
        if (legs[i].velprof->Vel(0.5*cycle_duration) > MAX_SWING_VEL) {
            not_enough_swing_time = true;
        }
    }

    if (not_enough_swing_time) center_legs();
    for (int i = (phase+1)%6; i < num_legs; i+=6) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
    }
    

    double dt = cycle_duration / TRAJ_RES;
    for (double t = 0.0; std::abs(t) < cycle_duration; t+=dt) {
        if(!is_ready || !is_power_on) return;
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB(legs[i].traj->Pos(t).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    is_in_null_pos = false;
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
    double dt = DEFAULT_TIMESTEP;

    double duration = 1.0;
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * 0.5, duration);
    }
    for (double t = 0.0; std::abs(t) < duration; t+=dt) {
        for (int i = (phase+1)%2; i < num_legs; i+=2) {
            legs[i].setPositionB(legs[i].traj->Pos(t).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }
    is_in_null_pos = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Kinematics>());
    rclcpp::shutdown();
    return 0;
}