#include "feature_tracking_contour_roi.hpp"
#include <rosinterface_handler/utilities.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>

#include <opencv2/opencv.hpp>
#include <rosinterface_handler/utilities.hpp>

#include "../commons/commons.hpp"

namespace viso_feature_tracking_ros_tool {

namespace cl = std::chrono;

// functions that are not declared in header
namespace {
///@brief release build assert
void Assert(bool condition, std::string error_message) {
    if (!condition) {
        throw std::runtime_error("in feature_tracking_contour_roi: " + error_message);
    }
}

feature_tracking::TrackerLibViso::Parameters GetVisoParams(const FeatureTrackingContourRoiInterface& params) {
    feature_tracking::TrackerLibViso::Parameters p;
    p.nms_n = params.nms_n;
    p.nms_tau = params.nms_tau;
    p.match_binsize = params.match_binsize;
    p.match_radius = params.match_radius;
    p.match_disp_tolerance = params.match_disp_tolerance;
    p.outlier_disp_tolerance = params.outlier_disp_tolerance;
    p.outlier_flow_tolerance = params.outlier_flow_tolerance;
    p.half_resolution = params.half_resolution;
    p.multi_stage = params.multi_stage;
    p.maxTracklength = params.max_tracklength;
    p.method = params.method;

    return p;
}

std::vector<std::vector<cv::Point>> convert_msg_to_contour(const opencv_apps::ContourArrayStamped& msg,
                                                           double scale_factor) {

    std::vector<std::vector<cv::Point>> out;
    out.reserve(msg.contours.size());

    for (const auto& c : msg.contours) {
        std::vector<cv::Point> cur_contour;
        cur_contour.reserve(c.points.size());
        for (const auto& p : c.points) {
            cur_contour.push_back(cv::Point(scale_factor * p.x, scale_factor * p.y));
        }
        out.push_back(cur_contour);
    }
    return out;
}
}

FeatureTrackingContourRoi::FeatureTrackingContourRoi(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    rosinterface_handler::setLoggerLevel(private_node_handle);
    params_.fromParamServer();

    /*
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&FeatureTrackingContourRoi::ReconfigureRequest, this, _1, _2));

    /*
     * Publishers & subscriber
     */

    publisher_ =
        private_node_handle.advertise<matches_msg_ros::MatchesMsg>(params_.publisher_msg_name, params_.msg_queue_size);
    {
        // subscriber setup

        subscriber1_ =
            std::make_unique<Subscriber1>(private_node_handle, params_.subscriber_contour, params_.msg_queue_size);
        subscriber2_ =
            std::make_unique<Subscriber2>(private_node_handle, params_.subscriber_image, params_.msg_queue_size);

        // setup synchronizer
        sync_ = std::make_unique<Synchronizer>(ApproximateTime(100), *(subscriber1_), *(subscriber2_));
        sync_->registerCallback(boost::bind(&FeatureTrackingContourRoi::Callback, this, _1, _2));
    }

    tracker_ = std::make_shared<feature_tracking::TrackerLibViso>(GetVisoParams(params_));

    rosinterface_handler::showNodeInfo();
}

/*
 * Use const ConstPtr for your callbacks.
 * The 'const' assures that you can not edit incoming messages.
 * The Ptr type guarantees zero copy transportation within nodelets.
 */
void FeatureTrackingContourRoi::Callback(const ContourMsg::ConstPtr& contours_msg,
                                         const sensor_msgs::Image::ConstPtr& image_msg) {

    auto start = cl::high_resolution_clock::now();
    // execute tracker and write matches
    cv_bridge::CvImagePtr cv_bridge_ptr = cv_bridge::toCvCopy(image_msg);

    ROS_DEBUG_STREAM("viso_feature_tracking: image_size=" << cv_bridge_ptr->image.rows << " x "
                                                          << cv_bridge_ptr->image.cols
                                                          << "\nscaling factor="
                                                          << params_.scale_factor);

    // image preprocessing
    if (params_.scale_factor != 1.0) {
        cv::Size imageSize = cv_bridge_ptr->image.size();
        cv::resize(cv_bridge_ptr->image,
                   cv_bridge_ptr->image,
                   cv::Size(imageSize.width * params_.scale_factor, imageSize.height * params_.scale_factor));
    }

    cv::GaussianBlur(
        cv_bridge_ptr->image, cv_bridge_ptr->image, cv::Size(params_.blur_size, params_.blur_size), params_.blur_sigma);

    // get mask from contours
    cv::Mat mask(cv_bridge_ptr->image.rows, cv_bridge_ptr->image.cols, CV_8UC1, cv::Scalar(0));

    if (contours_msg->contours.size() > 0) {
        auto contours = convert_msg_to_contour(*contours_msg, params_.scale_factor);
        //        for (size_t i = 0; i < contours.size(); ++i) {
        //            cv::drawContours(mask, contours, i, cv::Scalar(255), CV_FILLED, 8,
        //                             std::vector<cv::Vec4i>(), 0, cv::Point());
        //        }
        // take only first (biggest) contour
        cv::drawContours(mask, contours, 0, cv::Scalar(255), CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }
    // calc feature tracks
    tracker_->pushBack(cv_bridge_ptr->image, mask);
    tracker_->getTracklets(this->tracklets_);

    this->timestamps_.push_front(image_msg->header.stamp);

    auto end = cl::high_resolution_clock::now();
    int64_t duration = cl::duration_cast<cl::milliseconds>(end - start).count();
    ROS_INFO_STREAM("Duration feature tracking: " << duration << " ms");
    // publish
    Publish(this->timestamps_);

    // cut timestamps
    if (static_cast<int>(timestamps_.size()) > max_size_timestamps_)
        timestamps_.pop_back();
}

void FeatureTrackingContourRoi::Publish(const std::deque<ros::Time>& timestamps) {
    matches_msg_ros::MatchesMsg out_msg =
        commons::convert_tracklets_to_matches_msg(this->tracklets_, timestamps, params_.scale_factor);
    out_msg.header.stamp = timestamps.front();
    // out_msg->header.frame_id = config.frame_id;
    // out_msg->child_frame_id = config.base_link_frame_id;

    publisher_.publish(out_msg);
}

/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void FeatureTrackingContourRoi::ReconfigureRequest(FeatureTrackingContourRoiConfig& config, uint32_t level) {
    // save for comparing
    auto old_params = params_;

    params_.fromConfig(config);

    // reset tracker if params where altered
    if (WereTrackerParamsChanged(old_params)) {
        tracker_ = std::make_shared<feature_tracking::TrackerLibViso>(GetVisoParams(params_));
    }
}

bool FeatureTrackingContourRoi::WereTrackerParamsChanged(const FeatureTrackingContourRoiInterface& params) {
    bool unchanged = true;
    unchanged = unchanged && params_.nms_n == params.nms_n;
    unchanged = unchanged && params_.nms_tau == params.nms_tau;
    unchanged = unchanged && params_.match_binsize == params.match_binsize;
    unchanged = unchanged && params_.match_radius == params.match_radius;
    unchanged = unchanged && params_.match_disp_tolerance == params.match_disp_tolerance;
    unchanged = unchanged && params_.outlier_disp_tolerance == params.outlier_disp_tolerance;
    unchanged = unchanged && params_.outlier_flow_tolerance == params.outlier_flow_tolerance;
    unchanged = unchanged && params_.half_resolution == params.half_resolution;
    unchanged = unchanged && params_.multi_stage == params.multi_stage;
    unchanged = unchanged && params_.max_tracklength == params.max_tracklength;
    unchanged = unchanged && params_.method == params.method;

    return !unchanged;
}

} // namespace viso_feature_tracking_ros_tool
