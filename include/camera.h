#pragma once

#include <Eigen/Core>

#include "ray.h"

class Camera {
public:
    Camera(float fovy, const Eigen::Vector3f& eye, const Eigen::Vector3f& lookat, const Eigen::Vector3f& up, size_t width, size_t height, float z_near, float z_far);

    Ray generateRay(size_t x, size_t y);
private:
    Eigen::Matrix4f pers;
    size_t width, height;
};