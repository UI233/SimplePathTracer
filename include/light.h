#pragma once

#include <Eigen/Core>

#include "shape.h"

namespace simple_pt{

class Light {
public:
    virtual Eigen::Vector3f sampleLight() const = 0;
    virtual ~Light() {}
};

class MeshLight : public Light {
public:
    Eigen::Vector3f sampleLight() const override;
    MeshLight(const Mesh& mesh, const Eigen::Vector3f& radiance):
        m_mesh(mesh),
        m_radiance(radiance) 
        {}
private:
    Mesh m_mesh;
    Eigen::Vector3f m_radiance;
};

};
