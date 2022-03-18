#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <tinyxml2.h>

#include <algorithm>
#include <iostream>
#include <ctime>

#include "scene.h"
#include "integrator.h"
#include "helper.h"

namespace simple_pt
{
void Integrator::loadCamera(const std::string& config_file) {
    tinyxml2::XMLDocument doc; 
    doc.LoadFile(config_file.c_str());
    auto cam_info = doc.FirstChildElement("camera");
    if (cam_info != nullptr)
    {
        auto eye = str2vector3(cam_info->FirstChildElement("eye")->Attribute("value"));
        auto up = str2vector3(cam_info->FirstChildElement("up")->Attribute("value"));
        auto lookat = str2vector3(cam_info->FirstChildElement("lookat")->Attribute("value"));
        float fovy = cam_info->FirstChildElement("fovy")->FloatAttribute("value");
        int width = cam_info->FirstChildElement("width")->IntAttribute("value");
        int height = cam_info->FirstChildElement("height")->IntAttribute("value");
        m_cam = Camera(fovy, eye, lookat, up, width, height, 0.01f, 100.0f);
        m_frame.resize(height);
        for (auto& vec: m_frame) {
            vec.resize(width);
            std::fill(vec.begin(), vec.end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
        }
    }
    else
        throw "Invalid Camera Description";
}

//for debugging remove it
float max_dis = 0.0f;
void Integrator::render(Scene &scene) {
    for (int i = 0; i < m_cam.h(); ++i)
        for (int j = 0; j < m_cam.w(); ++j) {
            auto ray = m_cam.generateRay(j, i);
            m_frame[i][j] = pathTracing(ray, scene);
        }
}

void Integrator::draw() {
    std::vector<unsigned char> data(m_cam.w() * m_cam.h() * 3);
    for (int i = 0; i < m_cam.h(); ++i)
        for (int j = 0; j < m_cam.w(); ++j)
            for (int c = 0; c < 3; ++c)
                data[3 * (i * m_cam.w() + j) + c] = 255u - clamp<unsigned char>(m_frame[m_cam.h() - 1 - i][j][c] / max_dis * 255u, 0u, 255u);
    stbi_write_bmp("depth.bmp", m_cam.w(), m_cam.h(), 3, static_cast<void*>(data.data()));
}


Eigen::Vector3f Integrator::pathTracing(Ray ray, Scene &scene) const {
    auto pos = scene.intersect(ray).pos;
    if (pos.norm() == 0.0f)
        return pos;
    float dis = (pos - m_cam.o()).norm();
    max_dis = std::max(max_dis, dis);
    return Eigen::Vector3f(dis, dis, dis);
}
} // namespace simple_pt

#undef STB_IMAGE_WRITE_IMPLEMENTATION
