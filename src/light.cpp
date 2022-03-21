#include "light.h"

#include <iostream>

namespace simple_pt{
// Eigen::Vector3f MeshLight::sampleLight() const {
//     // todo: complete sampling method
//     return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
// }

Eigen::Vector3f MeshLight::lightEmitted(const igl::Hit& hit_on_light, const Eigen::Vector3f& dir) const {
    if (m_mesh->normal(hit_on_light).dot(dir) < 0.0f)
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    else
        return m_radiance;
}

TransmittedInfo MeshLight::sampleLight(const igl::Hit& hit_on_surface, const Eigen::Vector3f& pos) const {
    auto [vertex, normal, pdf] = m_mesh->uniformSampling();
    Eigen::Vector3f wi = (pos - vertex).normalized();
    float dis = (pos - vertex).norm();
    float pdf_solid = dis * dis / fabs(wi.dot(normal)) * m_mesh->pdf();
    if (normal.dot(wi) > 0.0f && !std::isinf(pdf_solid))
        return {Ray(vertex, wi), m_radiance, pdf_solid};
    else
        return {Ray(vertex, wi), Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0f};
}

float MeshLight::pdfLi(const Eigen::Vector3f& wi, const igl::Hit& hit_on_light, const Eigen::Vector3f& pos) const {
    auto normal = m_mesh->normal(hit_on_light);
    auto vertex = m_mesh->pos(hit_on_light);
    float dis = (pos - vertex).norm();
    // todo: validate it
    if (fabs(wi.dot(normal)) == 0.0f) {
        return 0.0f;
    }
    float pdf_solid = dis * dis / fabs(wi.dot(normal)) * m_mesh->pdf();
    if (std::isinf(pdf_solid)) {
        std::cout << dis  << " " << wi[0] << "," << wi[1] << ", " << wi[2] << " " << normal[0] << "," << normal[1] << "," <<normal[2] << std::endl; 
        throw "inf";
    }
    return pdf_solid;
}

};