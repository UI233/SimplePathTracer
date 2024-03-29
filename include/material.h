#pragma once

#include <tiny_obj_loader.h>

#include <igl/AABB.h>

#include <Eigen/Core>

#include <memory>
#include <variant>
#include <string>
#include <random>

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

/*! @enum MaterialFlag
    @brief to describle the type of material

    This distinction is used in integration to handle the special case that the distribution of material is dirac delta function
*/
enum MaterialFlag{
    BXDF_SPECULAR = 0x1,
    BXDF_DIFFUSE = 0x2,
    BXDF_REFRACTION = 0x4
};

/*! @class Material
    @brief the bsae class describle bsdf and distribution of samples attached to a shape
*/
class Material {
public:
    /**
     * Given a direction of outgoing ray, sample a direction of incoming ray
     * 
     * @param wo the outgoing direction of the ray
     * @param normal the normal of the hit point
     * @param hit the hit information
     * 
     * @return The ray, the scattering coeffient, and the probility density function.
     */
    virtual TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const = 0;
    /**
     * Given a direction of incoming ray, the normal of the surface, and the direction of outgoing ray,
     * 
     * return the spectrum of the ray carried
     * 
     * @param wi incoming direction
     * @param wo the outgoing direction of the ray
     * @param normal the normal of the hit point
     * @param hit The hit object that contains the intersection information.
     * 
     * @return The spectrum of the path.
     */
    virtual Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const = 0;
    /**
     * Given a direction of the incoming ray, and the normal of the surface, 
     * we calculate the probability density function of the ray hitting the surface.
     * 
     * @param wi incoming direction of the ray
     * @param wo the outgoing direction of the ray
     * @param normal the normal of the hit point
     * @param hit the hit information
     * 
     * @return The probability density function of the Phong model.
     */
    virtual float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const = 0;
    inline virtual int getFlag() const = 0;
    virtual ~Material() {}
};


/*! @class Lambert
    @brief the Lambertian diffuse reflection model
*/
class Lambert : public Material {
public:
    Lambert(const Eigen::Vector3f& kd) : m_kd(kd) {};
    TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    inline int getFlag() const override {return BXDF_DIFFUSE;}
private:
    Eigen::Vector3f m_kd;
};

/*! @class Phong
    @brief the modified Phong material in physically based rendering, this class handles a textured diffuse and a uniform specular term
*/
class Phong: public Material {
public:
    Phong(const tinyobj::material_t& material, const tinyobj::shape_t& shape, const tinyobj::attrib_t& attrib, const std::string& material_path="");
    TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    inline int getFlag() const override {return BXDF_DIFFUSE;}
private:
    Eigen::Vector3f getKd(const std::shared_ptr<igl::Hit>& hit) const;
    std::variant<Eigen::Vector3f, Texture> m_kd_map;
    std::variant<Eigen::Vector3f, Texture> m_ks_map;
    float m_ns, m_nd;
    const tinyobj::attrib_t& m_attrib;
    const tinyobj::shape_t&  m_shape;
    static std::uniform_real_distribution<float> m_distri;
    static std::default_random_engine m_rng;
};

/*! @class SpecularRefraction
    @brief a perfect specular material
*/
class SpecularRefraction: public Material {
public:
    SpecularRefraction(const float& ni): m_ni(ni) {}
    TransmittedInfo sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const override;
    Eigen::Vector3f f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const { return Eigen::Vector3f(0.0f, 0.0f, 0.0f);}
    float pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo,     * Given a direction of incoming light, the normal of the surface, and the direction of outgoing light,
     * 
     * return the color of the light
     * 
     * @param wi incoming light direction
     * @param wo the outgoing direction of the ray
     * @param normal the normal of the hit point
     * @param hit The hit object that contains the intersection information.
     * 
     * @return The color of the light.
     */ const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit = nullptr) const { return 0.0f;}
    inline int getFlag() const override {return BXDF_REFRACTION | BXDF_SPECULAR;}
private:
    float fresnel(const Eigen::Vector3f& wi, const Eigen::Vector3f& normal, float etat, float etai) const;
    TransmittedInfo sampleRefraction(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const;
    TransmittedInfo sampleReflection(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const;
    float m_ni;
    static std::uniform_real_distribution<float> m_distri;
    static std::default_random_engine m_rng;
};
}
