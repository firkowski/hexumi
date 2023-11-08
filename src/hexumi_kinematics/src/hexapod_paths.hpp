#pragma once
#include "kdl/path.hpp"
#include "kdl/rotational_interpolation.hpp"
#include "kdl/utilities/error.h"

#include <rclcpp/rclcpp.hpp>

#define INTEGRATION_RES 1000


namespace KDL {
    // rotates vector about abou an angle in the xy plane
    class Path_Arc : public Path
	{
		Frame start_pose;
        Vector rotcenter;
        double angle;
        RotationalInterpolation* orient;
		bool aggregate;

        double radius;
        double pathlength;
        int direction;

        bool test;

	public: // start pose, angle, 
		Path_Arc(const Frame&, 
                 const Vector&, 
                 double, 
                 bool, 
                 RotationalInterpolation*, bool);

		virtual double PathLength();
        virtual Frame Pos(double s) const;
		virtual Twist Vel(double s,double sd) const;
		virtual Twist Acc(double s,double sd,double sdd) const;
		virtual Path* Clone();
		virtual void Write(std::ostream& os);
        double LengthToS(double length);
        double getRadius() {return radius;}

		virtual IdentifierType getIdentifier() const {
			return ID_CIRCLE;
		}

        void setAngle(double);
        void makeTest() { test = true; }
        void makeNoTest() { test = false; }

        double getRadius() const { return radius; }

		virtual ~Path_Arc();
	};

    Path_Arc::Path_Arc(const Frame& _start_pose, const Vector& _rotcenter, double _angle, bool _test, RotationalInterpolation* _orient, bool _aggregate=true) :
        start_pose(_start_pose),
        rotcenter(_rotcenter),
        orient(_orient),
        aggregate(_aggregate),
        test(_test)
    {
        radius = (start_pose.p - rotcenter).Norm();
        if (radius < epsilon) {
            if (aggregate)
                delete orient;
            throw Error_MotionPlanning_Circle_ToSmall();
        }
        setAngle(_angle);
    }

    void Path_Arc::setAngle(double _angle) {
        if (_angle >= 0)
            direction = 1;
        else
            direction = -1;

        angle = std::abs(_angle);
        orient->SetStartEnd(start_pose.M, start_pose.M * Rotation::RotZ(direction * angle));
        if (angle * radius < epsilon && !test) {
            if (aggregate)
                delete orient;
            throw Error_MotionPlanning_Circle_No_Plane();
        }
        pathlength = angle * radius;
    }

    Path* Path_Arc::Clone() {
        return new Path_Arc(start_pose, rotcenter, direction * angle, test, orient->Clone(), true);
    }

    double Path_Arc::LengthToS(double length) {
	    return length;
    }

    double Path_Arc::PathLength() {
        return pathlength;
    }

    Frame Path_Arc::Pos(double s) const {
        double th = direction * s / radius;
        Frame rotframe = Frame(rotcenter);
        Vector start_pose_cor = start_pose.p - rotcenter;
        double p_x = start_pose_cor.x();
        double p_y = start_pose_cor.y();
        double p_z = start_pose_cor.z();
        Frame transform = Frame(Vector(-p_x*(1 - std::cos(th)) - p_y*std::sin(th),
                                        p_x*std::sin(th)       - p_y*(1 - std::cos(th)),
                                        0.0));
        return transform * start_pose;
    }

    Twist Path_Arc::Vel(double s,double sd) const {
        return Twist::Zero();
    }

    Twist Path_Arc::Acc(double s,double sd,double sdd) const {
        return Twist::Zero();
    }

    Path_Arc::~Path_Arc() {
        if (aggregate)
            delete orient;
    }

    void Path_Arc::Write(std::ostream& os) {
	os << "CIRCLE[ ";
	os << "  " << Pos(0) << std::endl;
	os << "  " << start_pose.p << std::endl;
	os << "  " << start_pose.M.UnitY() << std::endl;
	os << "  " << orient->Pos(pathlength) << std::endl;
	os << "  " << pathlength/radius/deg2rad << std::endl;
	os << "  ";orient->Write(os);
	os << "  " << radius;
	os << "]"<< std::endl;
}


    class Path_Cycloid : public Path {

        double height;
        double duration;

        Frame start_pose;
        Vector end_pos;

        double pathlength;

        Vector rolling_axis;
        double length;

        double alpha;
        double beta;

    public:
        Path_Cycloid (const KDL::Frame& _start_pose, 
                      const KDL::Vector& _end_pos, 
                      double _height, 
                      double _duration)
        : start_pose(_start_pose), end_pos(_end_pos), height(_height), duration(_duration) {
            rolling_axis = end_pos - start_pose.p;
            length = rolling_axis.Normalize();
            height = _height;
            duration = _duration;

            alpha = length / (2*M_PIl);
            beta  = height / 2;

            pathlength = modified_cycloid_arc_length(alpha, beta, INTEGRATION_RES);
        }

        virtual double PathLength() {return pathlength;}
        virtual Frame Pos(double s) const;
		virtual Twist Vel(double s, double sd) const;
		virtual Twist Acc(double s, double sd, double sdd) const;
		virtual Path* Clone();
		virtual void Write(std::ostream& os);
        double LengthToS(double length);

		virtual IdentifierType getIdentifier() const {
			return ID_CIRCLE;
		}        
    private:

        double modified_cycloid_arc_length(double alpha, double beta, double resolution) {
            long double delta_u = (2*M_PIl) / resolution;
            long double arc_length = 0.0;

            for (int i = 0; i < resolution; ++i) {
                // trapezoid rule
                long double u1 = i * delta_u;
                long double u2 = (i + 1) * delta_u;

                arc_length += 0.5 * ( sqrt( pow(alpha*(1 - std::cos(u1)), 2) + pow(beta*std::sin(u1), 2) ) +  
                                      sqrt( pow(alpha*(1 - std::cos(u2)), 2) + pow(beta*std::sin(u2), 2) ) ) * delta_u;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arc_length: %f", arc_length);
            return arc_length;
            }
    };

    Frame Path_Cycloid::Pos(double s) const {
        double u = s / pathlength;
        return Frame(Vector( (u - std::sin((2*M_PIl)*u) / (2*M_PIl)) * rolling_axis.x() * length, 
                             (u - std::sin((2*M_PIl)*u) / (2*M_PIl)) * rolling_axis.y() * length, 
                             (1 - std::cos((2*M_PIl)*u))                                * height / 2)) * start_pose;
    }

    Twist Path_Cycloid::Vel(double s, double sd) const {
        return Twist::Zero();
    }

    Twist Path_Cycloid::Acc(double s, double sd, double sdd) const {
        return Twist::Zero();
    }

    Path* Path_Cycloid::Clone() {
        return new Path_Cycloid(start_pose, end_pos, height, duration);
    }

    void Path_Cycloid::Write(std::ostream& os) {
        return;
    }

    double Path_Cycloid::LengthToS(double length) {
        return length;
    }
}

