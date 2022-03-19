#pragma once

#include <vector>
#include <random>

#include "mycamera.h"

namespace simple_pt
{
class Integrator {
public:
    Integrator(size_t num_samples = 240, size_t max_bounces = 10, size_t least_bounces = 3, float q = 0.5f):
        m_num_samples(num_samples),
        m_max_bounces(max_bounces),
        m_least_bounces(least_bounces),
        m_q(q) {}
    void loadCamera(const std::string& config_path);
    void render(const Scene& scene);
    void draw();
private:
    Camera m_cam;
    size_t m_width, m_height;
    std::vector<std::vector<Eigen::Vector3f>> m_frame;
    Eigen::Vector3f pathTracing(Ray ray, const Scene& scene) const;
    Eigen::Vector3f uniformSampleAllLights(const Scene &scene, const HitInfo& hit, const Ray& ray) const;
    // integrator parameter
    const size_t m_max_bounces, m_least_bounces;
    const float m_q;
    const size_t m_num_samples;
    // random number generator
    static std::random_device m_rng;
    static std::uniform_real_distribution<float> m_distri;
}; 
} // namespace simple_pt
