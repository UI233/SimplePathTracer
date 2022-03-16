#include <tinyxml2.h>

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
    }
    else
        throw "Invalid Camera Description";
}
} // namespace simple_pt
