#include "visualization.h"
#include <opencv/cv.hpp>

namespace feature_tracking {

namespace visualization {

void drawMatches(const feature_tracking::StereoTrackletList& tracklets,
                 const std::vector<std::pair<cv::Mat, cv::Mat>>& images,
                 cv::Mat& output,
                 int size,
                 int thickness,
                 int linetype) {
    if (images.empty()) {
        return;
    }
    if (images[0].first.empty()) {
        return;
    }
    int imCount = images.size();
    int imgWidth = images[0].first.cols;
    int imgHeight = images[0].first.rows;

    output = cv::Mat(imCount * imgHeight, 2 * imgWidth, CV_8UC3);
    for (int i = 0; i < imCount; i++) {
        int yOffset = i * imgHeight;
        const cv::Mat& thisLeft = images[i].first;
        const cv::Mat& thisRight = images[i].second;

        if (thisLeft.depth() != CV_8U || thisRight.depth() != CV_8U) {
            throw std::runtime_error("unsupported image format, only byte datatype supported.");
        }
        if (thisLeft.rows != imgHeight || thisRight.rows != imgHeight || thisLeft.cols != imgWidth ||
            thisRight.cols != imgWidth) {
            throw std::runtime_error("Image size differs from that of first image.");
        }

        cv::Mat thisLeftROI(output, cv::Rect(0, yOffset, imgWidth, imgHeight));
        if (thisLeft.channels() == 3) {
            thisLeft.copyTo(thisLeftROI);
        } else if (thisLeft.channels() == 1) {
            cv::cvtColor(thisLeft, thisLeftROI, CV_GRAY2BGR);
        } else {
            throw std::runtime_error("unsupported image format. Only grayscale or BGR images supported");
        }

        cv::Mat thisRightROI(output, cv::Rect(imgWidth, yOffset, imgWidth, imgHeight));
        if (thisRight.channels() == 3) {
            thisRight.copyTo(thisRightROI);
        } else if (thisRight.channels() == 1) {
            cv::cvtColor(thisRight, thisRightROI, CV_GRAY2BGR);
        } else {
            throw std::runtime_error("unsupported image format");
        }

        for (const feature_tracking::StereoTracklet& stereoTracklet : tracklets) {
            if (stereoTracklet.size() <= i) {
                // this tracklet doesn't exist in this image
                continue;
            }
            cv::RNG rng(stereoTracklet.id_);
            cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            cv::Point leftP(stereoTracklet[i].p1_.u_, stereoTracklet[i].p1_.v_);
            cv::Point rightP(stereoTracklet[i].p2_.u_, stereoTracklet[i].p2_.v_);

            cv::circle(thisLeftROI, leftP, size, color, thickness, linetype);
            cv::circle(thisRightROI, rightP, size, color, thickness, linetype);
        }
    }
}

} // namespace visualization

} // namespace feature_tracking
