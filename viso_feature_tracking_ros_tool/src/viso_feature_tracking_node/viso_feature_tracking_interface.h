/*
 * Copyright 2015. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#pragma once

// standard includes
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <ros/ros.h>

#include <feature_tracking_core/tracker_libviso.h>
#include "viso_feature_tracking_interface.h"
#include "viso_feature_tracking_parameters.h"

#include <sensor_msgs/Image.h>

namespace viso_feature_tracking_ros_tool {
class VisoFeatureTrackingInterface {

public:
    ///@brief ctor
    VisoFeatureTrackingInterface(ros::NodeHandle node_handle, ros::NodeHandle private_handle);

private: // attributes
    ///@brief ros stuff
    ros::Publisher publisher_matches_;
    ros::Subscriber subscriber_image_;

    ///@brief params for setting up the node
    Configuration config;
    Parameters viso_params;

    ///@brief store info that shall be published, from current to old
    feature_tracking::TrackletVector tracklets_;

    ///@brief timestamps storage for matches, descending order (from current to old)
    std::deque<ros::Time> timestamps_;

    ///@brief maximum size of timestamps_
    int max_size_timestamps_ = 1000;

private: // methods
    ///@brief process the input from ros, execute sampler, publish it as odometry
    void process(const sensor_msgs::ImageConstPtr& input);

    ///@brief publish odometry as rostopic
    void publish(const std::deque<ros::Time>& timestamps);

    ///@brief pointer to tracker
    std::shared_ptr<feature_tracking::TrackerLibViso> tracker;
};
}
