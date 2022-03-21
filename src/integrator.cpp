#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "omp.h"

#include <tinyxml2.h>

#include <algorithm>
#include <iostream>
#include <ctime>

#include "scene.h"
#include "integrator.h"
#include "helper.h"

namespace simple_pt
{
std::default_random_engine Integrator::m_rng;
std::uniform_real_distribution<float> Integrator::m_distri(0.0f, 1.0f);

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
        m_cam = Camera(fovy, eye, lookat, up, width, height, 0.1f, 500.0f);
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
void Integrator::render(const Scene &scene) {
    for (int i = 0; i < m_cam.h(); ++i)
        for (int j = 0; j < m_cam.w(); ++j) {
            // float offset_x = m_distri(m_rng), offset_y = m_distri(m_rng);
            auto ray = m_cam.generateRay(j, i);
            // #pragma omp parallel for
            // if (i == 43 && j == 59)
            for (int s = 0; s < m_num_samples; ++s)  {
                m_frame[i][j] += pathTracing(ray, scene);
            }
        }
}

void Integrator::draw() {
    auto tone_mapping_gamma = [](float c) {
        //todo: fix this
        return pow(1.0f - exp(-1.0f * c), 1.0 / 2.2);
    };
    std::vector<unsigned char> data(m_cam.w() * m_cam.h() * 3);
    for (int i = 0; i < m_cam.h(); ++i)
        for (int j = 0; j < m_cam.w(); ++j)
            for (int c = 0; c < 3; ++c) {
                if (std::isnan(m_frame[m_cam.h() - 1 - i][j][c])) {
                    std::cerr << m_cam.h() - 1 - i << " " << j << std::endl;
                    throw "nan";
                }
                // todo: simply draw out the image
                float mapped = clamp<float>(tone_mapping_gamma(m_frame[m_cam.h() - 1 - i][j][c] / m_num_samples), 0.0f, 1.0f);
                data[3 * (i * m_cam.w() + j) + c] = clamp<size_t>(static_cast<size_t>(mapped * 255u), 0u, 255u);
            }
    stbi_write_bmp("depth.bmp", m_cam.w(), m_cam.h(), 3, static_cast<void*>(data.data()));
}


Eigen::Vector3f Integrator::pathTracing(Ray ray, const Scene &scene) const {
    // test for ray
    // auto pos = scene.intersect(ray).pos;
    // if (pos.norm() == 0.0f)
    //     return pos;
    // float dis = (pos - m_cam.o()).norm();
    // max_dis = std::max(max_dis, dis);
    // return Eigen::Vector3f(dis, dis, dis);
    // 
    Eigen::Vector3f l(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f beta(1.0f, 1.0f, 1.0f);
    size_t bounce = 0;
    for (bounce = 0; bounce <= m_max_bounces; ++bounce) {
        auto intersect = scene.intersect(ray);
        auto pos = ray.at(intersect.hit.t);
        // sample 
        if (bounce == 0) {
            if (intersect.light) {
                l += intersect.light->lightEmitted(intersect.hit, -ray.m_t);
                break;
            }
            isnan(l, "light_emitted_nan");
        }
        else if (intersect.light)
            break;
        if (!intersect.shape)
            break;
        // sample light
        l += beta.cwiseProduct(uniformSampleAllLights(scene, intersect, ray));
        isnan(l, "beta_fail_nan");
        // sample the new direction
        Eigen::Vector3f normal = intersect.shape->normal(intersect.hit);
        // for debug
        auto transimitted_info = intersect.shape->sampleF(intersect.hit, ray);
        if (transimitted_info.possibility < 1e-7)
            break;
        beta = beta.cwiseProduct(transimitted_info.spectrum * fabs(normal.dot(transimitted_info.ray.m_t)) / transimitted_info.possibility);
        // randomlly terminate the process
        if (bounce >= m_least_bounces) {
            float q = std::max(m_q, 1.0f - beta[1]);
            if (m_distri(m_rng) < q)
                break;
            beta /= 1 - q;
        }
        isnan(beta, "rr_nan");
        ray = Ray(pos, transimitted_info.ray.m_t);
    }
    return l;
}

Eigen::Vector3f Integrator::uniformSampleAllLights(const Scene &scene, const HitInfo& intersect, const Ray& ray) const {
    Eigen::Vector3f l(0.0f, 0.0f, 0.0f);
    igl::Hit hit = intersect.hit;
    Eigen::Vector3f pos = ray.at(hit.t);
    for (size_t s; s < m_l_samples; ++s)
    for (auto& light : scene.getAllLights()) {
        // todo check sampling pos
        // sample on light source
        auto [wi, light_spectrum, light_pdf] = light->sampleLight(hit, pos); 
        // test for occulution
        auto visibility_hit = scene.intersect(wi).hit;
        auto normal = intersect.shape->normal(hit);
        if ((wi.at(visibility_hit.t) - pos).norm() < 1e-3 && light_spectrum.norm() > 1e-5 && light_pdf > 1e-4) {
            auto f = intersect.shape->getMaterial()->f(-wi.m_t, -ray.m_t, normal, std::make_shared<igl::Hit>(hit));
            float pdf = intersect.shape->getMaterial()->pdf(-wi.m_t, -ray.m_t, normal,  std::make_shared<igl::Hit>(hit));
            l += light_spectrum.cwiseProduct(f) / light_pdf * fabs(wi.m_t.dot(normal)) * powerHeuristic(light_pdf, 1, pdf, 1);
            if (isnan(l, "light_calc_nan"))
            {
                std::cout << light_pdf << " " << pdf << " " << f[0] <<"," <<f[1] <<"," << f[2] << " " << light_spectrum[0] << "," << light_spectrum[1] << ", " << light_spectrum[2] << std::endl;
                throw "error";
            }
        }
        
        // sample bsdf
        auto [wi_f, f, pdf] = intersect.shape->sampleF(intersect.hit, ray);
        f *= fabs(wi_f.m_t.dot(normal));
        Eigen::Vector3f le;
        if (pdf > 1e-4) {
            wi_f.m_o = pos;
            auto light_test = scene.intersect(wi_f);
            if (light_test.light == light) {
                // hit on current light
                light_pdf = light->pdfLi(-wi_f.m_t, light_test.hit, pos);
                if (light_pdf > 1e-4) {
                    le = light->lightEmitted(light_test.hit, -wi_f.m_t);
                    l += f.cwiseProduct(le) * powerHeuristic(pdf, 1, light_pdf, 1) / pdf;
                }
            }
        }
        if (isnan(l, "light_calc_nan"))
        {
            std::cout << light_pdf << " " << pdf << " " << f[0] <<"," <<f[1] <<"," << f[2] << " " << le[0] << "," << le[1] << ", " << le[2] << std::endl;
            throw "error";
        }
    }
    return l / static_cast<float>(m_l_samples);
}
} // namespace simple_pt

#undef STB_IMAGE_WRITE_IMPLEMENTATION
