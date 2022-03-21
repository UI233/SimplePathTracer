#pragma once

#include <Eigen/Core>

#include <algorithm>

namespace simple_pt{
using triangle_t = std::array<Eigen::Vector3f, 3>;

Eigen::Vector3f getUniformSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getUniformHemiSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getUnifromTriangleSample();

Eigen::Vector3f str2vector3(std::string s);

Eigen::Vector3f getCosineWeightHemiSphereSample(const Eigen::Vector3f& normal);

Eigen::Vector3f getSpecularWeight(const Eigen::Vector3f& specular, int n);

triangle_t indices2triangle(const std::array<size_t, 3>&, const std::vector<float>& );

template<typename T>
T clamp(T v, T l, T r) {
    if (v <= l)
        return l;
    else if (v >= r)
        return r;
    else
        return v;
}

bool isnan(Eigen::Vector3f &v, std::string msg);

float powerHeuristic(float pdf_a, int num_a, float pdf_b, int num_b);

float rgb2intensity(const Eigen::Vector3f& );
}
