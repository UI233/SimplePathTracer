#pragma once

#include <Eigen/Core>

#include "shape.h"

namespace simple_pt{

class Light {
public:
    virtual TransmittedInfo sampleLight(const igl::Hit& hit, const Eigen::Vector3f& pos) const = 0;
    virtual Eigen::Vector3f lightEmitted(const igl::Hit& hit, const Eigen::Vector3f& dir) const = 0;
    virtual ~Light() {}
};

class MeshLight : public Light {
public:
    TransmittedInfo sampleLight(const igl::Hit& hit, const Eigen::Vector3f& pos) const override;
    Eigen::Vector3f lightEmitted(const igl::Hit& hit, const Eigen::Vector3f& dir) const override;
    MeshLight(std::shared_ptr<Shape> mesh, const Eigen::Vector3f& radiance):
        m_mesh(mesh),
        m_radiance(radiance) 
        {}
private:
    std::shared_ptr<Shape> m_mesh;
    Eigen::Vector3f m_radiance;
};

};
