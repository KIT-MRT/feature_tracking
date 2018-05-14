#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "viso_feature_tracking_ros_tool/FeatureTrackingContourRoiInterface.h"

#include <feature_tracking_core/tracker_libviso.h>

#include <matches_msg_ros/MatchesMsg.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv_apps/ContourArrayStamped.h>

namespace viso_feature_tracking_ros_tool {

class FeatureTrackingContourRoi {
private:
    // types for sync subscriber
    using ContourMsg = opencv_apps::ContourArrayStamped;
    using Subscriber1 = message_filters::Subscriber<ContourMsg>;
    using Subscriber2 = message_filters::Subscriber<sensor_msgs::Image>;
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<ContourMsg, sensor_msgs::Image>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTime>;

public:
    FeatureTrackingContourRoi(ros::NodeHandle, ros::NodeHandle);

private: // publisher and subscriber
    ros::Publisher publisher_;

    ///@brief sync subscribers
    std::unique_ptr<Subscriber1> subscriber1_;
    std::unique_ptr<Subscriber2> subscriber2_;
    std::unique_ptr<Synchronizer> sync_;

private: // process method
    ///@brief process the input from ros, execute whatever, publish it
    void Callback(const ContourMsg::ConstPtr&, const sensor_msgs::Image::ConstPtr&);

    FeatureTrackingContourRoiInterface params_;

    dynamic_reconfigure::Server<FeatureTrackingContourRoiConfig> reconfigSrv_; // Dynamic reconfiguration service

    void ReconfigureRequest(FeatureTrackingContourRoiConfig&, uint32_t);

    ///@brief store info that shall be published, from current to old
    feature_tracking::TrackletVector tracklets_;

    ///@brief timestamps storage for matches, descending order (from current to old)
    std::deque<ros::Time> timestamps_;

    ///@brief maximum size of timestamps_
    int max_size_timestamps_ = 1000;

    ///@brief publish odometry as rostopic
    void Publish(const std::deque<ros::Time>& timestamps);

    ///@brief pointer to tracker
    std::shared_ptr<feature_tracking::TrackerLibViso> tracker_;

    ///@brief test if parameters of tracker have changed so that it needs to reset
    bool WereTrackerParamsChanged(const FeatureTrackingContourRoiInterface& params);

    ///@brief pointer to tracker
    std::shared_ptr<feature_tracking::TrackerLibViso> tracker;
};

} // namespace viso_feature_tracking_ros_tool
