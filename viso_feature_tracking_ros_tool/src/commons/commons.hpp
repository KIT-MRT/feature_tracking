#pragma once
#include <deque>
#include <feature_tracking/tracker_libviso.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <ros/ros.h>

namespace viso_feature_tracking_ros_tool {
namespace commons {

void Assert(bool condition, std::string error_message) {
    if (!condition) {
        throw std::runtime_error("in viso_feature_tracking/commons: " + error_message);
    }
}

matches_msg_ros::MatchesMsg convert_tracklets_to_matches_msg(feature_tracking::TrackletVector m,
                                                             const std::deque<ros::Time>& timestamps,
                                                             const double scale_factor = 1.0) {
    using MsgMatch = matches_msg_ros::FeaturePoint;
    using MsgTracklet = matches_msg_ros::Tracklet;

    matches_msg_ros::MatchesMsg out;

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
}
}
