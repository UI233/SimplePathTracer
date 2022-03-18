#pragma once

#include <vector>

#include "mycamera.h"

namespace simple_pt
{
class Integrator {
public:
    void loadCamera(const std::string& config_path);
    void render(Scene& scene);
    void draw();
private:
    Camera m_cam;
    size_t m_width, m_height;
    std::vector<std::vector<Eigen::Vector3f>> m_frame;

    Eigen::Vector3f pathTracing(Ray ray, Scene& scene) const;
}; 
} // namespace simple_pt
