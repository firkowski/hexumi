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
#include <limits>
#include <memory>

#define NUM_SAMPLES 200
#define MAX_DURATION 10

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
inline KDL::Vector EllipsoidWorkspace::center_l = KDL::Vector(0.0, 0.0, 0.0);
inline std::vector<double> EllipsoidWorkspace::axes = {0.0, 0.0, 0.0};


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
        duration = 0;
        setStationaryTrajectory();
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
    void setSegmentTrajectory(KDL::Path*, KDL::VelocityProfile*);

    void resetDuration() {duration = 0;}

    std::unique_ptr<KDL::Trajectory> traj;
    KDL::VelocityProfile* velprof = nullptr;
    
    static int tripodPhase;
    static int wavePhase;
    static int ripplePhase;

    double duration;

    static KDL::Twist motion_twist;
    static KDL::Frame body_pose;

private:

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
    float coxa_angle = 0.0f, femur_angle = 0.0f, tibia_angle = 0.0f;

    bool singularity_flag = false;

    bool is_active = true;

    EllipsoidWorkspace workspace;

    enum TrajType {support, transfer} traj_type;
    void setStationaryTrajectory();
    KDL::Path* line_path();
    KDL::Path* arc_path(KDL::Vector);
};

inline KDL::Twist Leg::motion_twist = KDL::Twist::Zero();
inline KDL::Frame Leg::body_pose = KDL::Frame::Identity();
inline int Leg::tripodPhase = 0;
inline int Leg::wavePhase = 0;
inline int Leg::ripplePhase = 0;

inline bool Leg::isInTaskspaceL(KDL::Vector p_l) {
    KDL::Vector vec = p_l - workspace.getCenterL();
    return (std::pow(vec.x(), 2) / std::pow(workspace.a()/2, 2) + 
            std::pow(vec.y(), 2) / std::pow(workspace.b()/2, 2) + 
            std::pow(vec.z(), 2) / std::pow(workspace.c()/2, 2)) <= 1;
}

inline bool Leg::isInTaskspaceB(KDL::Vector p_b) {
    return isInTaskspaceL(to_hip_frame(p_b));
}

inline bool Leg::setPositionL(KDL::Vector p_l) {
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

inline bool Leg::setPositionB(KDL::Vector p_b) {
    return setPositionL(to_hip_frame(p_b));
}

inline bool Leg::isInWorkspaceL(KDL::Vector p_l) {
    double L = std::hypot(p_l.x(), p_l.y()) - coxa_length;
    double reach = std::hypot(L, p_l.z());
    double yaw = std::atan2(p_l.y(), p_l.x());

    return (   reach <= tibia_length + femur_length
            && reach >= std::abs(tibia_length - femur_length)
            && yaw   <=  M_PI * 35.0 / 180.0
            && yaw   >= -M_PI * 35.0 / 180.0);
}

inline bool Leg::isInWorkspaceB(KDL::Vector p_b) {
    return isInWorkspaceL(to_hip_frame(p_b));
}

inline void Leg::setMotionParams(KDL::Twist _motion_twist, KDL::Frame _body_pose) {
    Leg::motion_twist = _motion_twist;
    Leg::body_pose = _body_pose;
}

inline void Leg::set_joint_angles() {
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
inline void Leg::setStationaryTrajectory() {
    velprof = nullptr;
    traj = std::make_unique<KDL::Trajectory_Stationary>(KDL::epsilon, KDL::Frame(current_position_b));
}

inline void Leg::setSegmentTrajectory(KDL::Path* path, KDL::VelocityProfile* profile) {
    profile->SetProfile(0.0, path->PathLength());
    velprof = profile;
    traj = std::make_unique<KDL::Trajectory_Segment>(path, profile);
}

inline void Leg::generateSupportPath() {
    traj_type = support;
    //body twist zero -> stationary
    if(KDL::Equal(motion_twist, KDL::Twist::Zero(), KDL::epsilon)) {
        setStationaryTrajectory();
        duration = 0;
        return;
    }

    if(!KDL::Equal(motion_twist.rot.z(), 0.0, KDL::epsilon)) {
        double wz = motion_twist.rot.z();
        KDL::Vector rotcenter_b = KDL::Vector(-motion_twist.vel.y() / wz,
                                               motion_twist.vel.x() / wz,
                                               current_position_b.z());
        double rho = (current_position_b - rotcenter_b).Norm();
        RCLCPP_DEBUG(rclcpp::get_logger("traj gen"), "leg %d CoR (%f, %f)", id, rotcenter_b.x(), rotcenter_b.y());
        // check if the rotation is not happening around one of the legs. only happens for specific v, omega, and placement of leg 1 o 4
        if (rho <= KDL::epsilon ) {
            setStationaryTrajectory();
            RCLCPP_DEBUG(rclcpp::get_logger("traj gen"), "leg %d path point, center of rotation", id);
            duration = MAX_DURATION;
            return;
        }
        KDL::Path* path = arc_path(rotcenter_b);
        if (path == nullptr) {
            setStationaryTrajectory();
            return;
        }
        setSegmentTrajectory(path, new KDL::VelocityProfile_Rectangular(std::abs(wz) * rho));
        duration = traj->Duration();
        RCLCPP_DEBUG(rclcpp::get_logger("traj gen"), "leg %d traj duration: %f", id, duration);
        return;
    }

    KDL::Path* path = line_path();
    if (path == nullptr) {
        setStationaryTrajectory();
        return;
    }
    setSegmentTrajectory(path, new KDL::VelocityProfile_Rectangular(motion_twist.vel.Norm()));
    duration = traj->Duration();
}

inline KDL::Path* Leg::line_path() {
    int eta = traj_type == support ? -1 : 1;

    double phi = std::atan2(motion_twist.vel.y(), motion_twist.vel.x());
    RCLCPP_DEBUG(rclcpp::get_logger("line_path()"), "phi: %f", phi);
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
        return nullptr;
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

inline KDL::Path* Leg::arc_path(KDL::Vector rotcenter) {
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
        RCLCPP_DEBUG(rclcpp::get_logger("arc gen"), "leg %d circle too small", id);
        duration = 0;
        return nullptr;
    } catch (const KDL::Error_MotionPlanning_Circle_No_Plane& e) {
        RCLCPP_DEBUG(rclcpp::get_logger("arc gen"), "leg %d circle no plane", id);
        duration = 0;
        return nullptr;
    }
}

// transfer trajectories
inline void Leg::generateTransferPathCenter(double height, double transfer_duration) {
    this->traj_type = transfer;
    KDL::Path* path = new KDL::Path_Cycloid(KDL::Frame(current_position_b), workspace.getCenterB(), height);
    double S = path->PathLength();
    if (S <= KDL::epsilon || transfer_duration <= KDL::epsilon) {
        delete path;
        setStationaryTrajectory();
        return;
    }
    setSegmentTrajectory(path, new KDL::VelocityProfile_Trap(2*S/transfer_duration, 4*S/std::pow(transfer_duration,2)));
}
