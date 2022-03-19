#include "material.h"
#include "helper.h"

#include <random>

namespace simple_pt {
TransmittedInfo Lambert::sample(const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const {
    Eigen::Vector3f wi = getCosineWeightHemiSphereSample(normal);
    // float pdf = 0.5f * M_1_PI;
    return {Ray(Eigen::Vector3f(0.0f, 0.0f, 0.0f), -wi), f(wi, wo, normal), pdf(wi, wo, normal)};
}

Eigen::Vector3f Lambert::f(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const {
    return m_ks * M_1_PI;
}

float Lambert::pdf(const Eigen::Vector3f& wi, const Eigen::Vector3f& wo, const Eigen::Vector3f& normal) const {
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

}