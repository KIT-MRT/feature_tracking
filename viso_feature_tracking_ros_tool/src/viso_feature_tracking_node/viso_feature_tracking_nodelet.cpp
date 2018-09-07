#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "viso_feature_tracking_interface.h"

namespace viso_feature_tracking_ros_tool{

class VisoFeatureTrackingNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<VisoFeatureTrackingInterface>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<VisoFeatureTrackingInterface> impl_;
};
}

PLUGINLIB_EXPORT_CLASS(viso_feature_tracking_ros_tool::VisoFeatureTrackingNodelet, nodelet::Nodelet);