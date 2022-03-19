#pragma once

#include <Eigen/Core>

namespace simple_pt{
Eigen::Vector3f getUniformSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getUniformHemiSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getUnifromTriangleSample();

Eigen::Vector3f str2vector3(std::string s);

Eigen::Vector3f getCosineWeightHemiSphereSample(const Eigen::Vector3f& normal);

template<typename T>
T clamp(T v, T l, T r) {
    if (v <= l)
        return l;
    else if (v >= r)
        return r;
    else
        return v;
}
}
