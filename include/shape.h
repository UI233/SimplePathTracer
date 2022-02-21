#pragma once

#include <Eigen/Core>

#include "ray.h"

class Shape {
public:
    virtual Eigen::Vector3f intersect(const Ray& r);
};