#pragma once

#include <Eigen/Core>

#include "utility.h"

class Material {
public:
    virtual TransmittedInfo sample(const Eigen::Vector3f& wo) const = 0;
};
