#pragma once

#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <variant>

#include <Eigen/Core>

#include <igl/AABB.h>

#include <tinyxml2.h>

#include <tiny_obj_loader.h>

#include "shape.h"
#include "light.h"
#include "mycamera.h"
#include "utility.h"
#include "ray.h"

namespace simple_pt
{
using tinyxml2::XMLDocument;

struct HitInfo {
    Eigen::Vector3f pos;
};

class Scene {
public:
    void load(const std::string& scene_model_path, const std::string& material_path, const std::string& scene_config_path);

    HitInfo intersect(const Ray& r) const;

    // todo: fill the lighting transimission info
    std::optional<TransmittedInfo> pickLight() const {
        return std::optional<TransmittedInfo>();
    }
private:
    // meta information
    igl::AABB<Eigen::MatrixXd, 3> m_tree;
    tinyobj::attrib_t m_attrib;
    std::vector<tinyobj::material_t> m_materials;
    std::vector<tinyobj::shape_t> m_shapes;
    // store customed information
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    // Camera m_cam;
    std::vector<std::vector<size_t>> m_material_group_faces;
    std::vector<std::shared_ptr<Light>> m_lights;
    std::vector<std::shared_ptr<Shape>> m_objects;
    void loadLight(const XMLDocument& doc);
    void loadScene(const std::string& scene_path, const std::string& material_path);
};
} // namespace simple_pt
