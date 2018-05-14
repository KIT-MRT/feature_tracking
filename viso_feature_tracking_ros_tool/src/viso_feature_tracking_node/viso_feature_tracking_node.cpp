#include <exception>
#include <ros/console.h>
#include "viso_feature_tracking_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "viso_feature_tracking_node");
    try {
        viso_feature_tracking_ros_tool::VisoFeatureTrackingInterface interface(ros::NodeHandle(), ros::NodeHandle("~"));
        ros::spin();
    } catch (std::runtime_error& e) {
        std::cerr << std::string("Failed to initialize viso_feature_tracking_ros_tool: ") << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return 0;
}