#pragma once

#include "ray.h"

#include <Eigen/Core>

namespace simple_pt{
struct TransmittedInfo {
    Ray ray;
    Eigen::Vector3f spectrum;
    float possibility;
};
};