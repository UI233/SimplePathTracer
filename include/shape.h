#pragma once
#include <Eigen/Core>

#include <tiny_obj_loader.h>

#include <igl/AABB.h>

#include <memory>
#include <random>

#include "ray.h"
#include "material.h"

namespace simple_pt{

struct SampleInfo {
    Eigen::Vector3f vertex;
    Eigen::Vector3f normal;
    float possibility;
};

class Shape {
public:
    virtual SampleInfo uniformSampling() const = 0;
    virtual TransmittedInfo sampleF(const igl::Hit& hit, const Ray& ray) const = 0;
    virtual Eigen::Vector3f normal(const igl::Hit& hit) const = 0;
    virtual std::shared_ptr<Material> getMaterial() const = 0;
    virtual ~Shape() {}
};

class Mesh: public Shape {
public:
    Mesh(size_t group_id, std::shared_ptr<Material> material, const tinyobj::attrib_t& attrib, const tinyobj::material_t& material_file, const tinyobj::shape_t& shape, const std::vector<size_t>& faces);
    SampleInfo uniformSampling() const override;
    // override for base class
    TransmittedInfo sampleF(const igl::Hit& hit, const Ray& ray) const override;
    Eigen::Vector3f normal(const igl::Hit& hit) const override;
    inline size_t getId() const {return m_group_id;}
    inline std::shared_ptr<Material> getMaterial() const override {return m_material;};
private:
    // todo: add texture
    // Eigen::Vector3f getTexture(const igl::Hit& hit) const;
private:
    size_t m_group_id;
    //todo: add some useful references to mesh data
    std::shared_ptr<Material> m_material;
    const tinyobj::attrib_t& m_attrib;
    const tinyobj::material_t& m_material_file;
    const tinyobj::shape_t& m_shape;
    const std::vector<size_t>& m_faces;
    float m_area;
    std::vector<float> m_area_accum;
    // for generating random samples
    static std::default_random_engine rng;
    static std::uniform_real_distribution<float> distri;
};
};