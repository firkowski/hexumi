#include "rclcpp/rclcpp.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

class GlobalParameterServer : public rclcpp::Node
{
public:
    GlobalParameterServer() : Node("global_parameter_server",
                                   rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)) {
        RCLCPP_INFO(this->get_logger(), "Global parameter server started.");

        //get relevant parameters
        get_parameter("num_legs", num_legs);

        std::string robot_description;
        if (!get_parameter("robot_description", robot_description) || robot_description.empty()) {
            throw std::runtime_error("Failed to retrieve robot_description from the parameter server.");
        }
        RCLCPP_INFO(this->get_logger(), "Successfully retrieved robot_description from the parameter server.");

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromString(robot_description, kdl_tree)) {
            throw std::runtime_error("Failed to parse URDF into KDL tree");
        }
        RCLCPP_INFO(this->get_logger(), "Successfully parsed URDF into KDL tree");

        get_parameter<std::vector<std::string>>("leg_suffixes", leg_suffixes);
        if (num_legs <= 0 || static_cast<int>(leg_suffixes.size()) != num_legs) {
            throw std::runtime_error("num_legs (" + std::to_string(num_legs) + ") must be positive and match the size of leg_suffixes (" +
                                     std::to_string(leg_suffixes.size()) + ")");
        }

        auto segment_frame = [&kdl_tree](const std::string& name) {
            auto segments_it = kdl_tree.getSegment(name);
            if (segments_it == kdl_tree.getSegments().end())
                throw std::runtime_error("URDF has no link named '" + name + "'");
            return segments_it->second.segment.getFrameToTip();
        };

        for (int i = 0; i < num_legs; i++) {
            hip_frames.push_back(segment_frame("coxa_" + leg_suffixes[i]));

            std::vector<double> origin = {hip_frames[i].p.x(), hip_frames[i].p.y(), hip_frames[i].p.z()};
            double yaw = std::atan2(origin[1], origin[0]);

            declare_parameter<std::vector<double>>(leg_suffixes[i] + "_origin", origin);

            declare_parameter(leg_suffixes[i] + "_yaw", rclcpp::ParameterValue(yaw));
        }
        //get coxa, femur, and tibia lengths from KDL Tree
        declare_parameter("coxa_length",  segment_frame("femur_" + leg_suffixes[0]).p.Norm());
        declare_parameter("femur_length", segment_frame("tibia_" + leg_suffixes[0]).p.Norm());
        declare_parameter("tibia_length", segment_frame("end_effector_" + leg_suffixes[0]).p.Norm());
    }
private:
    std::vector<std::string> leg_suffixes;
    std::vector<KDL::Frame> hip_frames;
    int num_legs = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<GlobalParameterServer>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("global_parameter_server"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}