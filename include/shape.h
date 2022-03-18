#pragma once
#include <Eigen/Core>

#include <tiny_obj_loader.h>

#include <memory>
#include <random>

#include "ray.h"
#include "material.h"

namespace simple_pt{

class Shape {
public:
    virtual Eigen::Vector3f uniformSampling() const = 0;
    virtual ~Shape() {}
};

class Mesh: public Shape {
public:
    Mesh(size_t group_id, std::shared_ptr<Material> material, const tinyobj::attrib_t& attrib, const tinyobj::material_t& material_file, const tinyobj::shape_t& shape, const std::vector<size_t>& faces);
    Eigen::Vector3f uniformSampling() const override;
    using triangle_t = std::array<Eigen::Vector3f, 3>;
private:
    triangle_t indices2triangle(const std::array<size_t, 3>&) const;
private:
    size_t m_group_id;
    //todo: add some useful references to mesh data
    const tinyobj::attrib_t& m_attrib;
    const tinyobj::material_t& m_material_file;
    const tinyobj::shape_t& m_shape;
    const std::vector<size_t>& m_faces;
    float m_area;
    std::shared_ptr<Material> m_material;
    std::vector<float> m_area_accum;
    // for generating random samples
    static std::random_device rng;
    static std::uniform_real_distribution<float> distri;
};
};