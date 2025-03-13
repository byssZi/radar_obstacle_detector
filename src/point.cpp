#include "radar_obstacle_detector/point.h"

namespace dbscan {

Point::Point(double x, double y, double z, double magnitude, double azimuth, double elevation, double rcs)
    : x_(x), y_(y), z_(z), magnitude_(magnitude), azimuth_(azimuth), elevation_(elevation), rcs_(rcs), state(PointState::unclassified) {}

double Point::EuclideanDistance(Point const& p) const {
    return std::pow(std::pow(x() - p.x(), 2) + std::pow(y() - p.y(), 2) + std::pow(z() - p.z(), 2) + std::pow(p.magnitude() - p.magnitude(), 2), 0.5);
}

bool Point::IsClassified() const {
    return state == PointState::classified; 
}

double Point::x() const {
    return x_;
}

double Point::y() const {
    return y_;
}

double Point::z() const {
    return z_;
}

double Point::magnitude() const {
    return magnitude_;
}

double Point::azimuth() const {
    return azimuth_;
}

double Point::elevation() const {
    return elevation_;
}

double Point::rcs() const {
    return rcs_;
}

}
