#include "viso_feature_tracking_interface.h"
#include <chrono>

#include <cv_bridge/cv_bridge.h>

#include <feature_tracking/tracker_libviso.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <rosinterface_handler/utilities.hpp>
//#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>

namespace viso_feature_tracking_ros_tool {

namespace cl = std::chrono;

using MatchesMsg = matches_msg_ros::MatchesMsg;

///@brief release build assert
void Assert(bool condition, std::string error_message) {
    if (!condition) {
        throw std::runtime_error("in viso_feature_tracking_ros_tool: " + error_message);
    }
}

VisoFeatureTrackingInterface::VisoFeatureTrackingInterface(ros::NodeHandle node_handle,
                                                           ros::NodeHandle private_node_handle) {
    rosinterface_handler::setLoggerLevel(private_node_handle);

    // read the configuration
    std::string configFileName;
    if (!private_node_handle.getParam("config", configFileName)) {
        throw std::runtime_error("No configuration specified");
    }
    config = VisoFeatureTrackingParameters(configFileName).get_config();
    viso_params = VisoFeatureTrackingParameters(configFileName).get_matcher_parameters();
    ROS_DEBUG_STREAM("tracklength=" << viso_params.maxTracklength);
    ROS_DEBUG_STREAM("refinement=" << viso_params.refinement);
    // instantiate tracker with config
    tracker = std::make_shared<feature_tracking::TrackerLibViso>(viso_params);

    /*
     * Publisher
     */
    publisher_matches_ =
        node_handle.advertise<matches_msg_ros::MatchesMsg>(config.matches_msg_name, config.matches_msg_queue_size);

    /*
     * subscriber setup
     */
    subscriber_image_ = node_handle.subscribe(
        config.image_msg_name, config.image_msg_queue_size, &VisoFeatureTrackingInterface::process, this);

    rosinterface_handler::showNodeInfo();
}

void VisoFeatureTrackingInterface::process(const sensor_msgs::ImageConstPtr& input) {
    auto start = cl::high_resolution_clock::now();
    // execute tracker and write matches
    cv_bridge::CvImagePtr cv_bridge_ptr = cv_bridge::toCvCopy(input);

    ROS_DEBUG_STREAM("viso_feature_tracking: image_size=" << cv_bridge_ptr->image.rows << " x "
                                                          << cv_bridge_ptr->image.cols
                                                          << "\nscaling factor="
                                                          << config.scale_factor);

    if (config.scale_factor != 1.0) {
        cv::Size imageSize = cv_bridge_ptr->image.size();
        cv::resize(cv_bridge_ptr->image,
                   cv_bridge_ptr->image,
                   cv::Size(imageSize.width * config.scale_factor, imageSize.height * config.scale_factor));
    }

    cv::GaussianBlur(cv_bridge_ptr->image,
                     cv_bridge_ptr->image,
                     cv::Size(viso_params.blur_size, viso_params.blur_size),
                     viso_params.blur_sigma);

    tracker->pushBack(cv_bridge_ptr->image);
    tracker->getTracklets(this->tracklets_, 0);

    this->timestamps_.push_front(input->header.stamp);

    auto end = cl::high_resolution_clock::now();
    int64_t duration = cl::duration_cast<cl::milliseconds>(end - start).count();
    ROS_INFO_STREAM("Duration feature matching and tracking: " << duration << " ms");
    // publish
    publish(this->timestamps_);

    // cut timestamps
    if (static_cast<int>(timestamps_.size()) > max_size_timestamps_) {
        timestamps_.pop_back();
    }
}

MatchesMsg convert_tracklets_to_matches_msg(feature_tracking::TrackletVector m,
                                            const std::deque<ros::Time>& timestamps,
                                            const double scale_factor = 1.0) {
    using MsgMatch = matches_msg_ros::FeaturePoint;
    using MsgTracklet = matches_msg_ros::Tracklet;

    MatchesMsg out;

    // assign matches with scaling
    out.tracks.reserve(m.size());

    // get maximum length of tracklet
    size_t max_length = 0;
    for (const auto& cur_tracklet : m) {
        MsgTracklet t;
        t.feature_points.reserve(cur_tracklet.size());

        // assign matches to tracklet msg, unscale if necessary
        for (const auto& cur_point : cur_tracklet) {
            MsgMatch msg_match;
            if (scale_factor != 1.0) {
                msg_match.u = float(cur_point.p1_.u_) / scale_factor;
                msg_match.v = float(cur_point.p1_.v_) / scale_factor;
            } else {
                msg_match.u = float(cur_point.p1_.u_);
                msg_match.v = float(cur_point.p1_.v_);
            }
            t.feature_points.push_back(msg_match);
        }
        Assert(t.feature_points.size() == cur_tracklet.size(), "inconsistent tracklet size");

        // assign unique id from tracker to be able to recognize the tracklets in different time
        // steps
        t.id = cur_tracklet.id_;

        // assign tracklet to output matches_msg
        out.tracks.push_back(t);

        // get maximum lenth
        if (t.feature_points.size() > max_length)
            max_length = t.feature_points.size();
    }

    // assign as many timestamps as we have measurements
    out.stamps.reserve(max_length);
    // timestamps.back() is the most recent so insert from last max_length elements
    out.stamps.insert(out.stamps.end(), timestamps.begin(), std::next(timestamps.begin(), max_length));
    ROS_DEBUG_STREAM("timestamps length=" << timestamps.size() << " max tracklet length=" << max_length
                                          << " out.stamps length="
                                          << out.stamps.size());

    //    Assert(std::next(out.stamps.rbegin()) == out.stamps.rend() ||
    //               (out.stamps.back() - *std::next(out.stamps.rbegin())).toSec() >= 0.,
    //           "timestamps in wrong order");

    if (out.stamps.size() > 1) {
        Assert((out.stamps.front() - *std::next(out.stamps.begin())).toSec() >= 0., "timestamps in wrong order");

        ROS_DEBUG_STREAM("viso_tracking: front()=" << out.stamps.front().toSec() << " next="
                                                   << std::next(out.stamps.begin())->toSec());
    }

    return out;
}

void VisoFeatureTrackingInterface::publish(const std::deque<ros::Time>& timestamps) {
    MatchesMsg out_msg = convert_tracklets_to_matches_msg(this->tracklets_, timestamps, config.scale_factor);
    out_msg.header.stamp = timestamps.front();
    // out_msg->header.frame_id = config.frame_id;
    // out_msg->child_frame_id = config.base_link_frame_id;

    publisher_matches_.publish(out_msg);
}

} // end of namespace
