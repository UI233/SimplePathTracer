#include <tiny_obj_loader.h>

#include <regex>
#include <string>
#include <map>

#include "helper.h"
#include "scene.h"

using namespace tinyxml2;

namespace simple_pt{
void Scene::loadLight(const XMLDocument& doc) {
    std::map<std::string, int> name_index;
    for (int i = 0; i < m_materials.size(); ++i) 
        name_index.insert(std::make_pair(m_materials[i].name, i));
    for (auto light = doc.FirstChildElement("light"); light != nullptr; light = light->NextSiblingElement("light")) {
        std::string name = light->Attribute("mtlname");
        Eigen::Vector3f radiance = str2vector3(light->Attribute("radiance"));
        // todo: insert a light to the list
        auto iter = name_index.find(name);
        if (iter != name_index.end()) {
            m_light_name.insert(std::make_pair(name, m_lights.size()));
            m_lights.push_back(std::make_shared<MeshLight>(
                    m_objects[iter->second], 
                    radiance
                )
            );
        }
    }
}

void Scene::loadScene(const std::string& scene_path, const std::string& material_path) {
    tinyobj::ObjReader reader;   
    tinyobj::ObjReaderConfig config;
    config.triangulate = true;
    config.mtl_search_path = material_path;
    reader.ParseFromFile(scene_path, config);
    m_materials = reader.GetMaterials();
    m_shapes = reader.GetShapes();
    m_attrib = reader.GetAttrib();
    // construct the tree in aabb tree
    V.resize(m_attrib.vertices.size() / 3, 3);
    F.resize(m_shapes[0].mesh.num_face_vertices.size(), 3);
    int j = 0;
    m_material_group_faces.resize(m_materials.size());
    for (auto& shape: m_shapes) {
        size_t offset = 0;
        for (auto v_num: shape.mesh.num_face_vertices) {
            for (size_t i = 0; i < v_num; ++i) 
                F(j, i) = shape.mesh.indices[offset + i].vertex_index;
            m_material_group_faces[shape.mesh.material_ids[j]].push_back(j);
            ++j;
            offset += v_num;
        }
    }

    for (int i = 0; i < m_attrib.vertices.size(); i += 3) {
        V.row(i / 3) << m_attrib.vertices[i], m_attrib.vertices[i + 1], m_attrib.vertices[i + 2];
    }
    // initialize aabb tree
    m_tree.init(V, F);
    // construct the list in a scene manager
    for (int i = 0; i < m_materials.size(); ++i) {
        // todo: support multiple types of materials
        Eigen::Vector3f ks(m_materials[i].diffuse);
        m_objects.push_back(std::make_shared<Mesh>(
                i, 
                std::make_shared<Phong>(m_materials[i], m_shapes[0], m_attrib, material_path),
                m_attrib,
                m_materials[i],
                m_shapes[0],
                m_material_group_faces[i]
            )
        );
    }
    // m_objects.push_back(std::make_shared)
}

void Scene::load(const std::string& scene_model_path, const std::string& scene_config_path, const std::string& material_path) {
    XMLDocument doc;
    doc.LoadFile(scene_config_path.c_str());
    // loadCamera(doc);
    loadScene(scene_model_path, material_path);
    loadLight(doc);
}

HitInfo Scene::intersect(const Ray& r) const {
    igl::Hit hit;
    Eigen::Vector3d o(r.m_o[0], r.m_o[1], r.m_o[2]), t(r.m_t[0], r.m_t[1], r.m_t[2]);
    // todo: return more information
    if (m_tree.intersect_ray(V, F, o + 1e-3 * t, t, hit)) {
        size_t idx = m_shapes[0].mesh.material_ids[hit.id];
        std::string name = m_materials[idx].name;
        if (m_light_name.count(name))
            return {
                hit,
                m_objects[idx],
                m_lights[m_light_name.find(name)->second]
            };
        else
            return {
                hit,
                m_objects[idx],
                nullptr
            };
    }
    return {hit, nullptr, nullptr};
}

}