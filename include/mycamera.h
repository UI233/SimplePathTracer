#pragma once

#include <Eigen/Core>

#include "ray.h"

namespace simple_pt{
class Camera {
public:
    Camera(float fovy, const Eigen::Vector3f& eye, const Eigen::Vector3f& lookat, const Eigen::Vector3f& up, size_t width, size_t height, float z_near, float z_far);

    Camera() = default;
    Ray generateRay(size_t x, size_t y) const;
private:
    Eigen::Matrix4f pers;
    Eigen::Vector3f origin;
    size_t width, height;
};
};