#pragma once

#include <Eigen/Core>

#include "utility.h"

namespace simple_pt {
class Material {
public:
    virtual TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const = 0;
    virtual Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const = 0;
    virtual float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const = 0;
    virtual ~Material() {}
};


class Lambert : public Material {
public:
    TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const override;
    Lambert(const Eigen::Vector3f& ks) : m_ks(ks) {};
    Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const override;
    float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const override;
private:
    Eigen::Vector3f m_ks;
};

// class Phong : public Material{
// public:
//     // Phong(float ks, float kd, float ns, float nd);

//     TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const override;
// private:
//     Eigen::Vector3f m_ks, m_kd;
//     float m_ns, m_nd;
//     float l_specular, l_diffuse;
//     static float masking(const Eigen::Vector3f& w, const Eigen::Vector3f& normal);
//     static float lambda(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal);
//     static float fresnel(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, float eta1, float eta2);

// };
}
