#pragma once

#include <Eigen/Core>

namespace simple_pt{
Eigen::Vector3f getUniformSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getUniformHemiSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getUnifromTriangleSample();

Eigen::Vector3f str2vector3(std::string s);
}
