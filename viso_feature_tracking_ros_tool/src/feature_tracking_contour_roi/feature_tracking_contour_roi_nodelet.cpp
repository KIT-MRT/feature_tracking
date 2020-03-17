#include "feature_tracking_contour_roi.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace viso_feature_tracking_ros_tool {

class FeatureTrackingContourRoiNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<FeatureTrackingContourRoi> m_;
};

void FeatureTrackingContourRoiNodelet::onInit() {
    m_.reset(new FeatureTrackingContourRoi(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace viso_feature_tracking_ros_tool

PLUGINLIB_EXPORT_CLASS(viso_feature_tracking_ros_tool::FeatureTrackingContourRoiNodelet,
                       nodelet::Nodelet);
