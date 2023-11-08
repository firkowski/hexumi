#pragma once

#include <kdl/frames.hpp>
#include <vector>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include "kdl/utilities/error.h"

#include "hexapod_paths.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>

#define ANGULAR_DEADZONE 0.02
#define NUM_SAMPLES 200
#define MAX_DURATION 10

double KDL::epsilon = 1e-9;

class Leg;
class EllipsoidWorkspace;

class EllipsoidWorkspace {
public: 
    EllipsoidWorkspace(KDL::Frame leg_origin) {center_b = leg_origin * center_l;};
    KDL::Vector getCenterL() const {return center_l;};
    KDL::Vector getCenterB() const {return center_b;};
    std::vector<double> getAxes() const {return axes;};
    double a() const {return axes[0];};
    double b() const {return axes[1];};
    double c() const {return axes[2];};

    static void setCenterL(KDL::Vector ctr) {center_l = ctr;};
    static void setAxes(std::vector<double> ax) {axes = ax;};

    static KDL::Vector center_l;
    static std::vector<double> axes;

private:
    KDL::Vector center_b;
};
KDL::Vector EllipsoidWorkspace::center_l = KDL::Vector(0.0, 0.0, 0.0); 
std::vector<double> EllipsoidWorkspace::axes = {0.0, 0.0, 0.0};


class Leg{
public:

    Leg(int id, 
         float coxa_length, float femur_length, float tibia_length,
         KDL::Frame T_origin, 
         KDL::Vector sitting_positionB)  : id(id),
                                          coxa_length(coxa_length),
                                          femur_length(femur_length),
                                          tibia_length(tibia_length),
                                          T_origin(T_origin),
                                          sitting_position_b(sitting_positionB),
                                          workspace(T_origin) {
        sitting_position_l = to_hip_frame(sitting_position_b);
        setPositionB(sitting_position_b);
        null_position_l = workspace.getCenterL();
        null_position_b = to_body_frame(null_position_l);

        traj_type = support;

        path = new KDL::Path_Point(KDL::Frame(KDL::Rotation::Identity(), current_position_b));
        velprof = new KDL::VelocityProfile_Rectangular(0.0);

        duration = 0;

        KDL::Frame current_frame_b = KDL::Frame::Identity();
        current_frame_b.p = current_position_b;
        traj = new KDL::Trajectory_Stationary(KDL::epsilon, current_frame_b);
    };

    ~Leg() {
        delete path;
        delete velprof;
        delete traj;
    };

    int getId() const {return id;};
    bool setPositionB(KDL::Vector);
    bool setPositionL(KDL::Vector);

    bool isInTaskspaceB(KDL::Vector);
    bool isInTaskspaceL(KDL::Vector);
    bool isInWorkspaceB(KDL::Vector);
    bool isInWorkspaceL(KDL::Vector);
    
    KDL::Vector getCurrentPositionB() const {return current_position_b;}
    KDL::Vector getCurrentPositionL() const {return current_position_l;}
    KDL::Vector getSittingPositionB() const {return sitting_position_b;}
    KDL::Vector getSittingPositionL() const {return sitting_position_l;}
    KDL::Vector getNullPositionB()    const {return null_position_b;}
    KDL::Vector getNullPositionL()    const {return null_position_l;}
    
    std::vector<float> getJointAngles() const {return current_joint_angles;}
    float getCoxaAngle()                const {return coxa_angle;}
    float getFemurAngle()               const {return femur_angle;}
    float getTibiaAngle()               const {return tibia_angle;}

    bool isSingular() const {return singularity_flag;}

    void activate()   {is_active = true;}
    void deactivate() {is_active = false;}
    bool isActive()   {return is_active;}

    //debug 
    EllipsoidWorkspace getWorkspace() const {return workspace;}

    static void setMotionParams(KDL::Twist, KDL::Frame);

    void generateSupportPath();
    void generateTransferPathCenter(double, double);

    void resetDuration() {duration = 0;}

    KDL::Path* path;
    KDL::VelocityProfile* velprof;
    KDL::Trajectory* traj;
    
    static int tripodPhase;
    static int wavePhase;
    static int ripplePhase;

    double duration;

    static KDL::Twist motion_twist;
    static KDL::Frame body_pose;

private:

    KDL::RotationalInterpolation_SingleAxis rotinterp;

    int id;
    float coxa_length, femur_length, tibia_length;
    KDL::Frame T_origin; // pose of leg's hip frame wrt. body frame
    KDL::Vector sitting_position_b;
    KDL::Vector sitting_position_l;

    KDL::Vector current_position_l;
    KDL::Vector current_position_b;
    KDL::Vector null_position_l;
    KDL::Vector null_position_b;

    KDL::Vector to_hip_frame(KDL::Vector p_b)  {return T_origin.Inverse(p_b);};
    KDL::Frame  to_hip_frame(KDL::Frame  T_b)  {return T_origin.Inverse() * T_b;};
    KDL::Vector to_body_frame(KDL::Vector p_l) {return T_origin*p_l;};
    KDL::Frame  to_body_frame(KDL::Frame  T_l) {return T_origin*T_l;};

    void set_joint_angles();
    std::vector<float> current_joint_angles;
    float coxa_angle, femur_angle, tibia_angle;

    bool singularity_flag;

    bool is_active;

    EllipsoidWorkspace workspace;

    enum TrajType {support, transfer} traj_type;
    KDL::Path* point_path();
    KDL::Path* line_path();
    KDL::Path* arc_path(KDL::Vector);
    static double path_length;
    static double path_angle;
};

double Leg::path_length = 0.0;
double Leg::path_angle = 0.0;

KDL::Twist Leg::motion_twist = KDL::Twist::Zero();
KDL::Frame Leg::body_pose = KDL::Frame::Identity();
int Leg::tripodPhase = 0;
int Leg::wavePhase = 0;
int Leg::ripplePhase = 0;

bool Leg::isInTaskspaceL(KDL::Vector p_l) {
    KDL::Vector vec = p_l - workspace.getCenterL();
    return (std::pow(vec.x(), 2) / std::pow(workspace.a()/2, 2) + 
            std::pow(vec.y(), 2) / std::pow(workspace.b()/2, 2) + 
            std::pow(vec.z(), 2) / std::pow(workspace.c()/2, 2)) <= 1;
}

bool Leg::isInTaskspaceB(KDL::Vector p_b) {
    return isInTaskspaceL(to_hip_frame(p_b));
}

bool Leg::setPositionL(KDL::Vector p_l) {
    if (isInWorkspaceL(p_l)) {
        current_position_l = p_l;
        current_position_b = to_body_frame(p_l);
        set_joint_angles();
        return true;
    }
    else {
        return false;
    }
}

bool Leg::setPositionB(KDL::Vector p_b) {
    return setPositionL(to_hip_frame(p_b));
}

bool Leg::isInWorkspaceL(KDL::Vector p_l) {
    float leg_radius = p_l.Norm();

    return (   leg_radius                     <= tibia_length + femur_length
            && leg_radius                     >= tibia_length - femur_length
            && std::atan2( p_l.y(), p_l.x() ) <= M_PI * 35.0 / 180.0
            && std::atan2( p_l.y(), p_l.x() ) >= -M_PI * 35.0 / 180.0);
}

bool Leg::isInWorkspaceB(KDL::Vector p_b) {
    return isInWorkspaceL(to_hip_frame(p_b));
}

void Leg::setMotionParams(KDL::Twist _motion_twist, KDL::Frame _body_pose) {
    Leg::motion_twist = _motion_twist;
    Leg::body_pose = _body_pose;
}

void Leg::set_joint_angles() {
    float x = current_position_l.x();
    float y = current_position_l.y();
    float z = current_position_l.z();

    if (std::abs(x*x + y*y) <= KDL::epsilon)
        singularity_flag = true; // coxa angle stays the same
    else {
        coxa_angle = std::atan2(y, x);
        singularity_flag = false;
    }

    float L = std::sqrt(x*x + y*y) - coxa_length;

    femur_angle = 
        std::atan2(z, L) + std::acos(   (std::pow(L, 2) + std::pow(z, 2) - std::pow(tibia_length, 2) + std::pow(femur_length, 2))
                                         / (2 * femur_length * std::sqrt(std::pow(L, 2) + std::pow(z, 2))) );

    tibia_angle =
        -std::acos(
                     (std::pow(L, 2) + std::pow(z, 2) - std::pow(tibia_length, 2) - std::pow(femur_length, 2))
                     / (2 * femur_length * tibia_length)
                  );

    current_joint_angles = {coxa_angle, femur_angle, tibia_angle};

}

// support trajectories
void Leg::generateSupportPath() {
    traj_type = support;
    //body twist zero -> stationary
    if(KDL::Equal(motion_twist, KDL::Twist::Zero(), KDL::epsilon)) {
        this->path = point_path();
        this->traj = new KDL::Trajectory_Stationary(KDL::epsilon, path->Pos(0));
        duration = 0;
        return;
    }

    //body twist.rot non-zero -> arc
    if(!KDL::Equal(motion_twist.rot, KDL::Vector::Zero(), KDL::epsilon)) {
        KDL::Vector rotcenter_b = KDL::Vector(0.0, 
                                           motion_twist.vel.x() / motion_twist.rot.z(),
                                           current_position_b.z());
        double rho = (current_position_b - rotcenter_b).Norm();
        RCLCPP_INFO(rclcpp::get_logger("traj gen"), "leg %d CoR %f", id, rotcenter_b.y());
        // check if the rotation is not happening around one of the legs. only happens for specific v, omega, and placement of leg 1 o 4
        if (rho <= KDL::epsilon ) {
            this->path = point_path();
            this->traj = new KDL::Trajectory_Stationary(KDL::epsilon, this->path->Pos(0));
            RCLCPP_INFO(rclcpp::get_logger("traj gen"), "leg %d path point, center of rotation", id);
            duration = MAX_DURATION;
            return;
        }
        this->path = arc_path(rotcenter_b);

        this->velprof = new KDL::VelocityProfile_Rectangular(motion_twist.rot.Norm() * rho);
        RCLCPP_INFO(rclcpp::get_logger("traj gen"), "leg %d support velocity: %f", id, motion_twist.rot.Norm() * rho);
        this->velprof->SetProfile(0.0, this->path->PathLength());
        this->traj = new KDL::Trajectory_Segment(this->path, this->velprof);
        duration = traj->Duration();
        RCLCPP_INFO(rclcpp::get_logger("traj gen"), "leg %d traj duration: %f", id, traj->Duration());
        return;
    }

    this->path = line_path();
    this->velprof = new KDL::VelocityProfile_Rectangular(motion_twist.vel.Norm());
    this->velprof->SetProfile(0.0, this->path->PathLength());
    this->traj = new KDL::Trajectory_Segment(this->path, this->velprof);
    duration = traj->Duration();
}

KDL::Path* Leg::point_path () {
    KDL::Frame pose = KDL::Frame(KDL::Rotation::Identity(), current_position_b);
    return new KDL::Path_Point(pose);
}

KDL::Path* Leg::line_path() {
    int eta = traj_type == support ? -1 : 1;

    double phi = std::atan2(motion_twist.vel.y(), motion_twist.vel.x());
    RCLCPP_INFO(rclcpp::get_logger("line_pat()"), "phi: %f", phi);
    double R = std::numeric_limits<double>::infinity();
    KDL::Frame start_pose = KDL::Frame(KDL::Rotation::Identity(), current_position_b);
    double max_dist = eta * std::max(workspace.a(), std::max(workspace.b(), workspace.c()));
    KDL::Frame max_end_pose = KDL::Frame(KDL::Rotation::Identity(),
                                           KDL::Vector(max_dist * cos(phi), 
                                                       max_dist * sin(phi),
                                                       0.0)) * start_pose;

    KDL::Path* test_path = new KDL::Path_Line(start_pose, max_end_pose, new KDL::RotationalInterpolation_SingleAxis(), R, true);
    if (test_path->PathLength() <= KDL::epsilon) {
        delete test_path;
        duration = 0;
        return point_path();
    }
    
    KDL::Frame end_pose = start_pose;
    double ds = test_path->PathLength() / NUM_SAMPLES;
    for (double s = 0.0; s < test_path->PathLength(); s += ds) {
        if (!isInTaskspaceB(test_path->Pos(s+ds).p)) {
            end_pose = test_path->Pos(s);
            break;
        }
    }

    delete test_path;

    return new KDL::Path_Line(start_pose, 
                              end_pose, 
                              new KDL::RotationalInterpolation_SingleAxis(), 
                              R, 
                              true);
}

KDL::Path* Leg::arc_path(KDL::Vector rotcenter) {
    int direction = motion_twist.rot.z() > 0 ? -1 : 1;

    KDL::Frame start_pose(current_position_b);
    start_pose.M = KDL::Rotation::RotZ(std::atan2(start_pose.p.y(), start_pose.p.x()));

    double max_angle = M_PI_2;

    try {
        KDL::Path_Arc path(start_pose, rotcenter, 0.0, true, new KDL::RotationalInterpolation_SingleAxis, true);
        double dalpha = direction * max_angle / NUM_SAMPLES;
        double alpha = 0.0;

        double ds = std::abs(dalpha) * path.getRadius();
        path.makeNoTest();

        while(true) {
            if(!isInTaskspaceB(path.Pos(path.PathLength() + ds).p) || std::abs(alpha) >= std::abs(max_angle)) {
                break;
            }
            alpha += dalpha;
            path.setAngle(alpha);
        }
        return path.Clone();
    } catch (const KDL::Error_MotionPlanning_Circle_ToSmall& e) {
        RCLCPP_INFO(rclcpp::get_logger("arc gen"), "leg %d circle too small", id);
        duration = 0;
        return point_path();
    } catch (const KDL::Error_MotionPlanning_Circle_No_Plane& e) {
        RCLCPP_INFO(rclcpp::get_logger("arc gen"), "leg %d circle no plane", id);
        duration = 0;
        return point_path();
    }
}

// transfer trajectories
void Leg::generateTransferPathCenter(double height, double duration) {
    this->traj_type = transfer;
    this->path = new KDL::Path_Cycloid(KDL::Frame(current_position_b), workspace.getCenterB(), height, duration);
    double S = this->path->PathLength();
    this->velprof = new KDL::VelocityProfile_Trap(2*S/duration, 4*S/pow(duration,2));
    this->velprof->SetProfile(0.0, this->path->PathLength());
    this->traj = new KDL::Trajectory_Segment(this->path, this->velprof);
}