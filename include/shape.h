#pragma once
#include "ray.h"
#include "material.h"

#include <Eigen/Core>

#include <memory>

namespace simple_pt{
class Shape {
public:
    virtual Eigen::Vector3f uniformSampling() const = 0;
    virtual ~Shape() {}
};

class Mesh: public Shape {
public:
    Mesh(size_t group_id, std::shared_ptr<Material> material):
        m_group_id(group_id),
        m_material(material) {}
    Eigen::Vector3f uniformSampling() const override;
private:
    size_t m_group_id;
    std::shared_ptr<Material> m_material;
};
};