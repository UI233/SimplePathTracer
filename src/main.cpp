#include "omp.h"

#include "fire-hpp/fire.hpp"

#include "scene.h"
#include "integrator.h"

int fireMain(std::string scene_model_path=fire::arg({0, "scene_path"}), 
    std::string scene_config_path=fire::arg({1, "config_path"}),
    std::string mat_path=fire::arg({2, "material_path"}), 
    size_t num_samples=fire::arg("--sample", 120u),
    size_t light_samples=fire::arg("--lsamples", 10u),
    size_t max_bounces=fire::arg("--bounce", 12u)
) {
    omp_set_num_threads(4);
    
    simple_pt::Scene scene;
    simple_pt::Integrator integrator(
        num_samples,
        light_samples,
        max_bounces
    );    
    scene.load(scene_model_path, scene_config_path, mat_path);
    integrator.loadCamera(scene_config_path);
    integrator.render(scene);
    integrator.draw(); 
    return 0;
}

FIRE(fireMain)