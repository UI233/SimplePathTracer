#include <algorithm>
#include <iostream>

#include "helper.h"
#include "shape.h"

namespace simple_pt
{
std::default_random_engine Mesh::rng;
std::uniform_real_distribution<float> Mesh::distri(0.0f, 1.0f);
Mesh::Mesh(size_t group_id, std::shared_ptr<Material> material, const tinyobj::attrib_t& attrib, const tinyobj::material_t& material_file, const tinyobj::shape_t& shape, const std::vector<size_t>& faces):
    m_group_id(group_id),
    m_material(material),
    m_attrib(attrib),
    m_material_file(material_file),
    m_shape(shape) ,
    m_faces(faces){
    m_area = 0.0f;
    for (auto face: faces) {
        triangle_t vertices = indices2triangle({
            m_shape.mesh.indices[3 * face].vertex_index,
            m_shape.mesh.indices[3 * face + 1].vertex_index,
            m_shape.mesh.indices[3 * face + 2].vertex_index
        }, m_attrib.vertices);
        Eigen::Vector3f v_diff0 = vertices[1] - vertices[0];
        Eigen::Vector3f v_diff1 = vertices[2] - vertices[0];
        m_area_accum.push_back(v_diff0.cross(v_diff1).norm() * 0.5f);
    }
    m_area = std::accumulate(m_area_accum.begin(), m_area_accum.end(), 0.0f);
    for (size_t i = 1; i < m_area_accum.size(); ++i)
        m_area_accum[i] += m_area_accum[i - 1];
}

SampleInfo Mesh::uniformSampling() const {
    float accum_area = distri(rng) * m_area;
    // todo: check this
    size_t id = std::lower_bound(m_area_accum.begin(), m_area_accum.end(), accum_area) - m_area_accum.begin();
    if (id == m_area_accum.size())
        --id;
    float a = distri(rng), b = distri(rng);
    float u = 1.0f - sqrtf(a), v = b * sqrtf(a);
    triangle_t vertices = indices2triangle({
            m_shape.mesh.indices[3 * m_faces[id]].vertex_index,
            m_shape.mesh.indices[3 * m_faces[id] + 1].vertex_index,
            m_shape.mesh.indices[3 * m_faces[id] + 2].vertex_index
    }, m_attrib.vertices);
    triangle_t normals = indices2triangle({
            m_shape.mesh.indices[3 * m_faces[id]].normal_index,
            m_shape.mesh.indices[3 * m_faces[id] + 1].normal_index,
            m_shape.mesh.indices[3 * m_faces[id] + 2].normal_index
    }, m_attrib.normals);
    Eigen::Vector3f pos = u * vertices[1] + v * vertices[2] + (1.0f - u - v) * vertices[0];
    Eigen::Vector3f normal = u * normals[1] + v * normals[2] + (1.0f - u - v) * normals[0];
    return {pos, normal.normalized(), 1.0f / m_area};
}

Eigen::Vector3f Mesh::normal(const igl::Hit& hit) const {
    std::array<size_t, 3> normal_indices{
        m_shape.mesh.indices[hit.id * 3].normal_index,
        m_shape.mesh.indices[hit.id * 3 + 1].normal_index,
        m_shape.mesh.indices[hit.id * 3 + 2].normal_index
    };
    auto normals = indices2triangle(normal_indices, m_attrib.normals);
    return (hit.u * normals[1] + hit.v * normals[2] + (1.0f - hit.u - hit.v) * normals[0]).normalized();
}

Eigen::Vector3f Mesh::pos(const igl::Hit& hit) const {
    std::array<size_t, 3> v_indices{
        m_shape.mesh.indices[hit.id * 3].vertex_index,
        m_shape.mesh.indices[hit.id * 3 + 1].vertex_index,
        m_shape.mesh.indices[hit.id * 3 + 2].vertex_index
    };
    auto v = indices2triangle(v_indices, m_attrib.vertices);
    return (hit.u * v[1] + hit.v * v[2] + (1.0f - hit.u - hit.v) * v[0]).normalized();

}


TransmittedInfo Mesh::sampleF(const igl::Hit& hit, const Ray& ray) const {
    // todo: add normal computation
    Eigen::Vector3f normal_v = normal(hit);
    igl::Hit temp = hit;
    // todo: add tex color
    return m_material->sample(-ray.m_t, normal_v, std::make_shared<igl::Hit>(temp));
}
} // namespace simple_pt
