#include "rclcpp/rclcpp.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

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
        if (!get_parameter("robot_description", robot_description)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to retrieve robot_description from the parameter server.");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved robot_description from the parameter server.");
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromString(robot_description, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Successfully parsed URDF into KDL tree");
        }

        get_parameter<std::vector<std::string>>("leg_suffixes", leg_suffixes);
        std::map<std::string,KDL::TreeElement>::const_iterator segments_it;
        for (int i = 0; i < num_legs; i++) {
            segments_it = kdl_tree.getSegment("coxa_" + leg_suffixes[i]);
            hip_frames.push_back((*segments_it).second.segment.getFrameToTip());

            std::vector<double> origin = {hip_frames[i].p.x(), hip_frames[i].p.y(), hip_frames[i].p.z()};
            double yaw = std::atan2(origin[1], origin[0]);

            declare_parameter<std::vector<double>>(leg_suffixes[i] + "_origin", origin);

            declare_parameter(leg_suffixes[i] + "_yaw", rclcpp::ParameterValue(yaw));
        }
        //get coxa, femur, and tibia lengths from KDL Tree
        segments_it = kdl_tree.getSegment("femur_" + leg_suffixes[0]);
        double coxa_length = (*segments_it).second.segment.getFrameToTip().p.Norm();
        declare_parameter("coxa_length", coxa_length);

        segments_it = kdl_tree.getSegment("tibia_" + leg_suffixes[0]);
        double femur_length = (*segments_it).second.segment.getFrameToTip().p.Norm();
        declare_parameter("femur_length", femur_length);

        segments_it = kdl_tree.getSegment("end_effector_" + leg_suffixes[0]);
        double tibia_length = (*segments_it).second.segment.getFrameToTip().p.Norm();
        declare_parameter("tibia_length", tibia_length);

    }
private:
    std::vector<std::string> leg_suffixes;
    std::vector<KDL::Frame> hip_frames;
    int num_legs;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalParameterServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}