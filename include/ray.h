#pragma once

#include <Eigen/Core>

class Ray {
public:
    Ray(const Eigen::Vector3f& o, const Eigen::Vector3f& t);
    Eigen::Vector3f o, t;
};