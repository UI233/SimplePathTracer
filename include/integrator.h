#pragma once

#include "mycamera.h"

namespace simple_pt
{
class Integrator {
public:
    void loadCamera(const std::string& config_path);
private:
    Camera m_cam;
}; 
} // namespace simple_pt
