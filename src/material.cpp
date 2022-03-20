#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <random>
#include <cstdlib>

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
    constexpr float factor = 1.0f / 255.0f;
    return Eigen::Vector3f(
        static_cast<float>(data[3 * (y * m_width + x)]) * factor,
        static_cast<float>(data[3 * (y * m_width + x) + 1]) * factor,
        static_cast<float>(data[3 * (y * m_width + x) + 2]) * factor
    );
}

Eigen::Vector3f Texture::at(float x, float y) const {
    int x0 = static_cast<int>(std::floor(x * m_width) + 5e-1), x1 = x0 + 1;
    int y0 = static_cast<int>(std::floor(y * m_height) + 5e-1), y1 = y0 + 1;
    double x0_float = static_cast<float>(x0) / m_width, x1_float = static_cast<float>(x1) / m_width;
    double y0_float = static_cast<float>(y0) / m_height, y1_float = static_cast<float>(y1) / m_height;
    Eigen::Vector3f f00 = get(x0, y0), f01 = get(x0, y1), f10 = get(x1, y0), f11 = get(x1, y1); 
    Eigen::Vector3f fr1 = ((x1_float - x) * f01 + (x - x0_float) * f11) / (x1_float - x0_float);
    Eigen::Vector3f fr0 = ((x1_float - x) * f00 + (x - x0_float) * f10) / (x1_float - x0_float);
    Eigen::Vector3f res =((y1_float - y) * fr1 + (y - y0_float) * fr0) / (y1_float - y0_float);
    assert(!isnan(res));
    return res;
}

TransmittedInfo Lambert::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>&) const {
    Eigen::Vector3f wi = getCosineWeightHemiSphereSample(normal);
    // float pdf = 0.5f * M_1_PI;
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), wi), f(wi, wo, normal), pdf(wi, wo, normal)};
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

// float Phong::masking(const Eigen::Vector3f& w, const Eigen::Vector3f& normal) {
//     constexpr float alpha = 0.5f;
//     float cos2theta = w.dot(normal) * w.dot(normal);
//     float sin2theta = 1.0f - cos2theta;
//     float tan2theta = sin2theta / cos2theta;
//     if (std::isinf(tan2theta) || std::isnan(tan2theta))
//         return 0.0f;
//     float cos4theta = cos2theta * cos2theta;
//     float e = ((cos2theta / (alpha * alpha)) + (sin2theta / (alpha * alpha))) * tan2theta;
//     return 1.0f / (M_PI * alpha * alpha * cos4theta * (1.0f + e) * (1.0f + e));
// }

// float Phong::lambda(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) {
//     return 1.0f / (1.0f + masking(wi, normal), + masking(wo, normal));
// }

// float Phong::fresnel(const Eigen::Vector3f& wi, const Eigen::Vector3f& normal, float etat, float etai) {
//     float cosi = wi.dot(normal);
//     float sini = std::sqrt(std::max(0.0f, 1.0f - cosi * cosi));
//     float sint = sini * etat / etai; 
//     float cost = std::sqrt(std::max(0.0f, 1.0f - sint * sint));

//     float rpar1 = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
//     float rpar2 = ((etai * cost) - (etat * cosi)) / ((etai * cost) + (etat * cosi));
//     return 0.5f * (rpar1 * rpar1 + rpar2 * rpar2);
// }

// // Phong::Phong(float t_ks, float t_kd, float t_ns, float t_nd):
// //     ks(t_ks * 0.5f),
// //     kd(t_kd * 0.5f),
// //     ns(t_ns),
// //     nd(t_nd),
// //     l_diffuse(1.0f / M_PI),
// //     l_specular((t_ns + 2.0f) /(3.1415926 * 2.0f)) {
// //     }

// TransmittedInfo Phong::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const {
//     float costheta = wo.dot(normal);
//     Eigen::Vector3f wi = getUniformHemiSphereSample(normal);

//     float specular_index = pow(normal.dot((0.5f * (-wo + wi)).normalized()), m_ns);
//     Eigen::Vector3f spectrum = m_kd * l_diffuse + specular_index * m_ks * l_specular;
//     float possibility = 1.0f / (2.0f * M_PI);
//     if (m_nd != 1.0f) {
//         // todo: refraction
//     }

//     return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), -wi), spectrum, possibility};
// }


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
    nd = material.shininess;
    ns = material.ior;
}


TransmittedInfo Phong::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit) const {
    Eigen::Vector3f wi = getCosineWeightHemiSphereSample(normal);
    // float pdf = 0.5f * M_1_PI;
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), wi), f(wi, wo, normal, hit), pdf(wi, wo, normal)};
}

Eigen::Vector3f Phong::f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>& hit) const {
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
        assert(!isnan(kd));
    }
    return kd * M_1_PI;
}

float Phong::pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal, const std::shared_ptr<igl::Hit>&) const {
    if (normal.dot(wo) * normal.dot(wi) >= 0.0f)
        return fabs(wi.dot(normal)) * M_1_PI;
    else
        return 0.0f;
} 
}
#undef STB_IMAGE_IMPLEMENTATION
