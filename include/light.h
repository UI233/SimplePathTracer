#pragma once

#include <Eigen/Core>

#include "shape.h"

namespace simple_pt{

class Light {
public:
    virtual TransmittedInfo sampleLight(const igl::Hit& hit, const Eigen::Vector3f& pos) const = 0;
    virtual Eigen::Vector3f lightEmitted(const igl::Hit& hit, const Eigen::Vector3f& dir) const = 0;
    virtual float pdfLi(const Eigen::Vector3f& wi, const igl::Hit& hit_on_light, const Eigen::Vector3f& pos) const = 0;
    virtual ~Light() = default;
};

class MeshLight : public Light {
public:
    TransmittedInfo sampleLight(const igl::Hit& hit, const Eigen::Vector3f& pos) const override;
    Eigen::Vector3f lightEmitted(const igl::Hit& hit, const Eigen::Vector3f& dir) const override;
    float pdfLi(const Eigen::Vector3f& wi, const igl::Hit& hit_on_light, const Eigen::Vector3f& pos) const override;
    MeshLight(std::shared_ptr<Shape> mesh, const Eigen::Vector3f& radiance):
        m_mesh(mesh),
        m_radiance(radiance) 
        {}
private:
    /*! @var m_mesh
        @brief the mesh descripting the shape of area light
    */
    std::shared_ptr<Shape> m_mesh;
    Eigen::Vector3f m_radiance;
};

};
