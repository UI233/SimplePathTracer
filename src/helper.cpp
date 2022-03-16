#include "helper.h"

#include <random>
#include <cmath>
#include <regex>

namespace simple_pt
{
Eigen::Vector3f getUniformSphereSample(const Eigen::Vector3f& normal) {
    static std::random_device rng;
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
} // namespace simple_pt
