#include <algorithm>

#include "shape.h"

namespace simple_pt
{
std::random_device Mesh::rng;
std::uniform_real_distribution<float> Mesh::distri(0.0f, 1.0f);

Mesh::triangle_t Mesh::indices2triangle(const std::array<size_t, 3>& indices) const {
    triangle_t res;
    for (int i = 0; i < 3; ++i)
        res[i] = Eigen::Vector3f(
            m_attrib.vertices[3 * indices[i]],
            m_attrib.vertices[3 * indices[i] + 1],
            m_attrib.vertices[3 * indices[i] + 2]
        );
    return res;
}

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
        });
        Eigen::Vector3f v_diff0 = vertices[1] - vertices[0];
        Eigen::Vector3f v_diff1 = vertices[2] - vertices[0];
        m_area_accum.push_back(v_diff0.cross(v_diff1).norm());
    }
    m_area = std::accumulate(m_area_accum.begin(), m_area_accum.end(), 0.0f);
    for (size_t i = 1; i < m_area_accum.size(); ++i)
        m_area_accum[i] += m_area_accum[i - 1];
}

Eigen::Vector3f Mesh::uniformSampling() const {
    float accum_area = distri(rng) * m_area_accum.size();
    size_t id = std::lower_bound(m_area_accum.begin(), m_area_accum.end(), accum_area) - m_area_accum.begin();
    float a = distri(rng), b = distri(rng);
    float u = 1.0f - sqrtf(a), v = b * sqrtf(a);
    triangle_t vertices = indices2triangle({
            m_shape.mesh.indices[3 * m_faces[id]].vertex_index,
            m_shape.mesh.indices[3 * m_faces[id] + 1].vertex_index,
            m_shape.mesh.indices[3 * m_faces[id] + 2].vertex_index
    });
    return u * vertices[0] + v * vertices[1] + (1.0f - u - v) * vertices[2];
}

} // namespace simple_pt
