#pragma once

#include <Eigen/Core>

namespace simple_pt{
struct TransmittedInfo {
    Eigen::Vector3f wi;
    Eigen::Vector3f spectrum;
    float possibility;
};
};