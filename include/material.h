#pragma once

#include <tiny_obj_loader.h>

#include <igl/AABB.h>

#include <Eigen/Core>

#include <memory>
#include <variant>
#include <string>

#include "utility.h"

namespace simple_pt {
class Texture {
public:
    Texture(const std::string& filename);
    Texture():data(nullptr) {}
    Texture(const Texture& other):
        m_width(other.m_width),
        m_height(other.m_height),
        m_channels(other.m_channels) {
        std::copy(other.data, other.data + m_channels * m_width * m_height * sizeof(unsigned char), data);
    } 
    ~Texture();
    Eigen::Vector3f at(float x, float y) const;
private:
    int m_width, m_height;
    int m_channels;
    unsigned char* data;
    Eigen::Vector3f get(int x, int y) const;
};

class Material {
public:
    virtual TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const = 0;
    virtual Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const = 0;
    virtual float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const = 0;
    virtual ~Material() {}
};


class Lambert : public Material {
public:
    TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    Lambert(const Eigen::Vector3f& kd) : m_kd(kd) {};
private:
    Eigen::Vector3f m_kd;
};

class Phong: public Material {
public:
    Phong(const tinyobj::material_t& material, const tinyobj::shape_t& shape, const tinyobj::attrib_t& attrib, const std::string& material_path="");
    TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
private:
    std::variant<Eigen::Vector3f, Texture> m_kd_map;
    std::variant<Eigen::Vector3f, Texture> m_ks_map;
    float ns, nd;
    const tinyobj::attrib_t& m_attrib;
    const tinyobj::shape_t&  m_shape;
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
