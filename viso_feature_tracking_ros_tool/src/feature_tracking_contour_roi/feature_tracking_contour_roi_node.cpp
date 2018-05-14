#include "feature_tracking_contour_roi.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "feature_tracking_contour_roi_node");

    viso_feature_tracking_ros_tool::FeatureTrackingContourRoi feature_tracking_contour_roi(ros::NodeHandle(),
                                                                                           ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
