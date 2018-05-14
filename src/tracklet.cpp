#include "tracklet.h"

namespace feature_tracking {
u_int64_t Tracklet::nextId = 0;
u_int64_t StereoTracklet::nextId = 0;

ImagePoint::ImagePoint(float u, float v, int index) : u_(u), v_(v), index_(index) {
    ;
}


Match::Match(ImagePoint p) : p1_(p), x_(nullptr) {
    ;
}

Match::Match(float u, float v, int index) : p1_(u, v, index), x_(nullptr) {
    ;
}


StereoMatch::StereoMatch(ImagePoint p1, ImagePoint p2) : Match(p1), p2_(p2) {
    ;
}

StereoMatch::StereoMatch(float u1, float v1, int index1, float u2, float v2, int index2)
        : Match(u1, v1, index1), p2_(u2, v2, index2) {
    ;
}


Tracklet::Tracklet() : std::deque<Match>(), id_(Tracklet::nextId++), age_(0) {
    ;
}


StereoTracklet::StereoTracklet() : std::deque<StereoMatch>(), id_(StereoTracklet::nextId++), age_(0) {
    ;
}
}
