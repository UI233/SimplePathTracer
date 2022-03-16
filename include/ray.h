#pragma once

#include <Eigen/Core>

namespace simple_pt{
class Ray {
public:
    Ray(const Eigen::Vector3f& o, const Eigen::Vector3f& t):
    m_o(o),
    m_t(t) {}
    Eigen::Vector3f m_o, m_t;
};
};