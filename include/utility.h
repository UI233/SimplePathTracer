#pragma once

#include <Eigen/Core>

struct TransmittedInfo {
    Eigen::Vector3f wi;
    Eigen::Vector3f spectrum;
    float possibility;
};