#pragma once

#include <Eigen/Core>

class Light {
public:
    virtual Eigen::Vector3f sampleLight() const = 0;
};
