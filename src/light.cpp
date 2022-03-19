#include "light.h"

namespace simple_pt{
// Eigen::Vector3f MeshLight::sampleLight() const {
//     // todo: complete sampling method
//     return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
// }

Eigen::Vector3f MeshLight::lightEmitted(const igl::Hit& hit_on_light, const Eigen::Vector3f& dir) const {
    if (m_mesh->normal(hit_on_light).dot(dir) > 0.0f)
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    else
        return m_radiance;
}

TransmittedInfo MeshLight::sampleLight(const igl::Hit& hit_on_surface, const Eigen::Vector3f& pos) const {
    auto [vertex, normal, pdf] = m_mesh->uniformSampling();
    Eigen::Vector3f wi = (pos - vertex).normalized();
    float dis = (pos - vertex).norm();
    float pdf_solid = dis * dis / fabs(wi.dot(normal)) * pdf;
    if (normal.dot(wi) >= 0.0f)
        return {Ray(vertex, wi), m_radiance, pdf_solid};
    else
        return {Ray(vertex, wi), Eigen::Vector3f(0.0f, 0.0f, 0.0f), pdf_solid};
}

};