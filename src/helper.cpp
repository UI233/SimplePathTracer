#include "helper.h"

#include <Eigen/Eigen>

#include <random>
#include <iostream>
#include <cmath>
#include <regex>

namespace simple_pt
{
Eigen::Vector3f getUniformSphereSample(const Eigen::Vector3f& normal) {
    static std::default_random_engine rng;
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    float theta = dist(rng) * 2.0f * M_PI, phi = dist(rng) * M_PI;
    return Eigen::Vector3f(cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi));
}

Eigen::Vector3f getUniformHemiSphereSample(const Eigen::Vector3f& normal) {
    auto sample = getUniformSphereSample(normal);
    float dot = sample.dot(normal);
    if (dot < 0.0f)
        sample -= 2.0f * dot * normal;
    return sample;
}

Eigen::Vector3f getCosineWeightHemiSphereSample(const Eigen::Vector3f& normal) {
    static std::default_random_engine rng;
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    Eigen::Vector3f x_v = normal.cross(Eigen::Vector3f(1.0, 0.0, 0.0));
    if (x_v.norm() < 1e-6)
        x_v = normal.cross(Eigen::Vector3f(0.0, 1.0, 0.0)).normalized();
    Eigen::Vector3f y_v = normal.cross(x_v);
    float r = sqrtf(dist(rng)), theta = M_PI * 2.0f * dist(rng);
    float x = r * cos(theta), y = r * sin(theta);
    float z = (std::max(0.0f, 1.0f - x * x - y * y));
    return x * x_v + y * y_v + z * normal;
}

Eigen::Vector3f getSpecularWeight(const Eigen::Vector3f& normal, int n) {
    static std::default_random_engine rng;
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    Eigen::Vector3f x_v = normal.cross(Eigen::Vector3f(1.0, 0.0, 0.0));
    if (x_v.norm() < 1e-6)
        x_v = normal.cross(Eigen::Vector3f(0.0, 1.0, 0.0)).normalized();
    Eigen::Vector3f y_v = normal.cross(x_v);
    float eta = dist(rng), eta1 = dist(rng);
    float sqrt_eta = sqrt(std::max(0.0, 1.0 - pow(eta, 2.0f / (n + 1)))), angle = 2.0f * M_PI * eta1;
    float x = sqrt_eta * cos(angle), y = sqrt_eta * sin(angle), z = pow(eta, 1.0f / (n + 1));
    return x * x_v + y * y_v + z * normal;
}

Eigen::Vector3f str2vector3(std::string s) {
    std::regex pattern("[0-9]+(\\.[0-9]+)?");
    std::smatch m;
    Eigen::Vector3f ret(0.0, 0.0, 0.0);
    int i = 0;
    while (std::regex_search (s, m, pattern)) {
        m.str();
        ret[i++] = std::atof(m.str().c_str());
        s = m.suffix().str();
        if (i == 3)
            break;
    }
    return ret;
}

triangle_t indices2triangle(const std::array<size_t, 3>& indices, const std::vector<float>& vertices) {
    triangle_t res;
    for (int i = 0; i < 3; ++i)
        res[i] = Eigen::Vector3f(
            vertices[3 * indices[i]],
            vertices[3 * indices[i] + 1],
            vertices[3 * indices[i] + 2]
        );
    return res;
}

bool isnan(Eigen::Vector3f &v, std::string msg) {
    if( std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2]))
    {
        std::cout << msg << std::endl;
        return true;
    }
    return false;
}

float powerHeuristic(float pdf_a, int num_a, float pdf_b, int num_b) {
    float fa = pdf_a * num_a, fb = pdf_b * num_b;
    return (fa * fa) / (fa * fa + fb * fb);
}

float rgb2intensity(const Eigen::Vector3f& rgb) {
    return 0.299 * rgb[0] + 0.587 * rgb[1] + rgb[2] * 0.114;
}
} // namespace simple_pt
