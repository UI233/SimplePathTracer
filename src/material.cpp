#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <random>
#include <cstdlib>
#include <iostream>

#include "material.h"
#include "helper.h"

namespace simple_pt {

Texture::Texture(const std::string& filename) {
    data = stbi_load(filename.c_str(), &m_width, &m_height, &m_channels, 3);
}

Texture::~Texture() {
    free(data);
}

Eigen::Vector3f Texture::get(int x, int y) const {
    x %= m_width;
    y %= m_height;
    if (x < 0)
        x += m_width;
    if (y < 0)
        y += m_height;
    constexpr float factor = 1.0f / 255.0f;
    return Eigen::Vector3f(
        static_cast<float>(data[3 * (y * m_width + x)]) * factor,
        static_cast<float>(data[3 * (y * m_width + x) + 1]) * factor,
        static_cast<float>(data[3 * (y * m_width + x) + 2]) * factor
    );
}

/**
 * Given a point in the texture, return the color using bilinear interpolation
 * 
 * @param x The x coordinate of the point in the texture.
 * @param y the y coordinate of the point in the texture.
 * 
 * @return The interpolated value of the texture at the given coordinates.
 */
Eigen::Vector3f Texture::at(float x, float y) const {
    int x0 = static_cast<int>(std::floor(x * m_width) + 5e-1), x1 = x0 + 1;
    int y0 = static_cast<int>(std::floor(y * m_height) + 5e-1), y1 = y0 + 1;
    double x0_float = static_cast<float>(x0) / m_width, x1_float = static_cast<float>(x1) / m_width;
    double y0_float = static_cast<float>(y0) / m_height, y1_float = static_cast<float>(y1) / m_height;
    Eigen::Vector3f f00 = get(x0, y0), f01 = get(x0, y1), f10 = get(x1, y0), f11 = get(x1, y1); 
    Eigen::Vector3f fr1 = ((x1_float - x) * f01 + (x - x0_float) * f11) / (x1_float - x0_float);
    Eigen::Vector3f fr0 = ((x1_float - x) * f00 + (x - x0_float) * f10) / (x1_float - x0_float);
    Eigen::Vector3f res =((y1_float - y) * fr1 + (y - y0_float) * fr0) / (y1_float - y0_float);
    assert(!isnan(res, "Ds"));
    return res;
}

TransmittedInfo Lambert::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>&) const {
    Eigen::Vector3f wi = getCosineWeightHemiSphereSample(normal);
    // float pdf = 0.5f * M_1_PI;
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), wi.normalized()), f(wi, wo, normal), pdf(wi, wo, normal)};
}


Eigen::Vector3f Lambert::f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>&) const {
    return m_kd * M_1_PI;
}

float Lambert::pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>&) const {
    if (normal.dot(wo) * normal.dot(wi) >= 0.0f)
        return fabs(wi.dot(normal)) * M_1_PI;
    else
        return 0.0f;
} 

std::uniform_real_distribution<float> Phong::m_distri(0.0f, 1.0f);
std::default_random_engine Phong::m_rng;

Phong::Phong(const tinyobj::material_t& material, const tinyobj::shape_t& shape, const tinyobj::attrib_t& attrib, const std::string& material_path):
    m_attrib(attrib),
    m_shape(shape) {
    if (material.diffuse_texname.length())
        m_kd_map.emplace<Texture>(material_path + "/" + material.diffuse_texname);
    else
        m_kd_map.emplace<Eigen::Vector3f>(material.diffuse);
    if (material.specular_texname.length())
        m_ks_map.emplace<Texture>(material.specular_texname);
    else
        m_ks_map.emplace<Eigen::Vector3f>(material.specular);
    m_ns = material.shininess;
    /* Setting the refractive index of the material. */
    m_nd = material.ior;
}

/**
 * If the material has a texture, then the kd is the color at the texture coordinate of the hit point.
 * Otherwise, the kd is the constant color of the material
 * 
 * @param hit the hit information of the ray-mesh intersection
 * 
 * @return The Kd color of the hit point.
 */
Eigen::Vector3f Phong::getKd(const std::shared_ptr<igl::Hit>& hit) const {
    Eigen::Vector3f kd;
    if (m_kd_map.index() == 0)
        kd = std::get<Eigen::Vector3f>(m_kd_map);
    else {
        const Texture& tex = std::get<Texture>(m_kd_map);
        // calculate vt
        if (hit == nullptr)
            throw "Not provided hit info";
        std::array<size_t, 3> indices {
            m_shape.mesh.indices[3 * hit->id].texcoord_index,
            m_shape.mesh.indices[3 * hit->id + 1].texcoord_index,
            m_shape.mesh.indices[3 * hit->id + 2].texcoord_index
        };

        std::array<Eigen::Vector2f, 3> uvs {
            Eigen::Vector2f(m_attrib.texcoords[2 * indices[0]], m_attrib.texcoords[2 * indices[0] + 1]),
            Eigen::Vector2f(m_attrib.texcoords[2 * indices[1]], m_attrib.texcoords[2 * indices[1] + 1]),
            Eigen::Vector2f(m_attrib.texcoords[2 * indices[2]], m_attrib.texcoords[2 * indices[2] + 1])
        };
        Eigen::Vector2f uv = hit->u * uvs[1] + hit->v * uvs[2] + (1.0f - hit->u - hit->v) * uvs[0];
        kd = tex.at(uv[0], uv[1]);
    }
    return kd;
}

/**
 * Given a direction of outgoing ray, sample a direction of incoming ray
 * 
 * @param wo the outgoing direction of the ray
 * @param normal the normal of the hit point
 * @param hit the hit information
 * 
 * @return The ray, the f value, and the pdf value.
 */
TransmittedInfo Phong::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit) const {
    float ks_intensity = rgb2intensity(std::get<Eigen::Vector3f>(m_ks_map));
    float kd_intensity = rgb2intensity(getKd(hit));
    float total_inv = 1.0 / (ks_intensity + kd_intensity);
    float kd_pdf = kd_intensity * total_inv;
    Eigen::Vector3f wi;
    // do {
        if (m_distri(m_rng) <= kd_pdf || ks_intensity < 1e-5)
        {
            wi = getCosineWeightHemiSphereSample(normal);
        }
        else {
            Eigen::Vector3f h_normal = normal.dot(wo) * normal;
            Eigen::Vector3f wr = (h_normal - wo) + h_normal;
            wi = getSpecularWeight(wr, m_ns);
        }
        wi = wi.normalized();
    // } while (wi.dot(normal) < 1e-6);
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), wi), f(wi, wo, normal, hit), pdf(wi, wo, normal, hit)};
}

Eigen::Vector3f Phong::f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit) const {
    Eigen::Vector3f kd = getKd(hit); 
    Eigen::Vector3f ks;
    if (normal.dot(wi) < 0.0f)
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (m_ks_map.index() == 0)
        ks = std::get<Eigen::Vector3f>(m_ks_map);
    if (ks.norm() > 1e-6) {
        Eigen::Vector3f total = ks + kd;
        if (total[0] >= 1.0f || total[1] >= 1.0f || total[2] >= 1.0f) {
            ks = 0.5 * ks;
            kd = 0.5 * kd;
        }
        Eigen::Vector3f h_normal = normal.dot(wi) * normal;
        Eigen::Vector3f wr = (h_normal - wi) + h_normal;
        float cosa = fabs(wo.dot(wr.normalized()));
        // 
        return kd * M_1_PI + ks * (m_ns + 2) / (2.0f * M_PI) * pow(cosa, m_ns);
    }
    else
        return kd * M_1_PI;
}

float Phong::pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit) const {
    float ks_intensity = rgb2intensity(std::get<Eigen::Vector3f>(m_ks_map));
    if (normal.dot(wo) * normal.dot(wi) < 0.0f)
        return 0.0f;
    if  (ks_intensity > 1e-5) {
        float kd_intensity = rgb2intensity(getKd(hit));
        float total_inv = 1.0 / (ks_intensity + kd_intensity);
        float kd_pdf = kd_intensity * total_inv, ks_pdf = ks_intensity * total_inv;
        Eigen::Vector3f h_normal = normal.dot(wo) * normal;
        Eigen::Vector3f wr = (h_normal - wo) + h_normal;

        // if (normal.dot(wo) * normal.dot(wi) >= 0.0f) {
        return kd_pdf * fabs(wi.dot(normal)) * M_1_PI + ks_pdf * pow(fabs(wr.dot(wi)), m_ns) * (m_ns + 1) * M_1_PI * 0.5f;
        // }
    }
    return fabs(wi.dot(normal)) * M_1_PI ;
} 

std::uniform_real_distribution<float> SpecularRefraction::m_distri(0.0f, 1.0f);
std::default_random_engine SpecularRefraction::m_rng;

/**
 * Given a direction of the incident ray, a normal to the surface, and the indices of refraction of the
 * incident and the medium, 
 * it returns the reflection coefficient
 * 
 * @param wo the outgoing ray direction
 * @param normal the normal vector of the surface being hit
 * @param etat the refractive index of the material on the outside of the surface
 * @param etai The refractive index of the material on the side of the surface from which the ray is
 * coming.
 * 
 * @return The fresnel value.
 */
float SpecularRefraction::fresnel(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, float etat, float etai) const {
    // todo : remove it after debuggin
    // return 1.0f;
    if (wo.dot(normal) < 0)
        std::swap(etat, etai);
    float cosi = fabs(wo.dot(normal));
    float sini = std::sqrt(std::max(0.0f, 1.0f - cosi * cosi));
    float sint = sini * etai / etat; 
    if (sint >= 1.0f)
        return 1.0f;
    float cost = std::sqrt(std::max(0.0f, 1.0f - sint * sint));

    float rpar1 = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    float rpar2 = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    // float r0 = (etai - etat) * (etai - etat) / ((etai + etat) * (etai + etat));
    // return r0 + (1.0f - r0) * pow(1.0f - std::max(1.0f, fabs(wo.dot(normal))), 5.0f);
    return 0.5f * (rpar1 * rpar1 + rpar2 * rpar2);
}

TransmittedInfo SpecularRefraction::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit) const {
    float fr = fresnel(wo, normal, 1.0f, m_ni);
    float q = m_distri(m_rng);
    // std::cout << "fr:" << fr <<std::endl;
    if (q < fr) {
        auto info = sampleReflection(wo, normal);
        info.possibility *= fr;
        info.spectrum *= fr;
        return info;
    }
    else {
        auto info = sampleRefraction(wo, normal);
        info.possibility *= 1.0f - fr;
        info.spectrum *= 1.0f - fr;
        return info;
    }
}


/**
 * Given a ray and a normal, sample a new ray that is refracted
 * 
 * @param wo the incoming ray direction
 * @param normal the normal vector of the surface
 * 
 * @return The ray, the color, and the probability of the ray.
 */
TransmittedInfo SpecularRefraction::sampleRefraction(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const {
    float etao = 1.0f, etai = m_ni;
    //determine the direction of ray
    if (wo.dot(normal) < 0)
        std::swap(etao, etai);
    float cos_wo = wo.dot(normal), sin_wo =sqrtf(std::max(0.0f, 1.0f - cos_wo * cos_wo));
    float sin_wi = sin_wo * etao / etai, cos_wi = sqrtf(std::max(0.0f, 1.0f - sin_wi * sin_wi));
    Eigen::Vector3f wi;
    if (sin_wi >= 1.0f) {
        // total reflection
        Eigen::Vector3f hi = wo - normal * wo.dot(normal);
        wi = -hi + wo.dot(normal) * normal;
    }
    else {
        Eigen::Vector3f hi = wo - normal * wo.dot(normal);
        hi = -hi.normalized();
        Eigen::Vector3f normal_i = wo.dot(normal) < 0 ? normal : -normal;
        wi = cos_wi * normal_i + sin_wi * hi;
    }
    float intensity = (etai * etai) / (etao * etao);
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), wi.normalized()), Eigen::Vector3f(1.0f, 1.0f, 1.0f) / fabs(wi.dot(normal)) * intensity, 1.0f};
}

/**
 * Sample a ray that is reflected from the surface
 * 
 * @param wo The incoming ray direction
 * @param normal the normal vector of the surface at the point of reflection.
 * 
 * @return The ray, the color, and the probability of the ray.
 */
TransmittedInfo SpecularRefraction::sampleReflection(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const {
    Eigen::Vector3f h_normal = normal.dot(wo) * normal;
    Eigen::Vector3f wi = (h_normal - wo) + h_normal;
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), wi.normalized()), Eigen::Vector3f(1.0f, 1.0f, 1.0f) / fabs(wi.dot(normal)), 1.0f};
}
}
#undef STB_IMAGE_IMPLEMENTATION
