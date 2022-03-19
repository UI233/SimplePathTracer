#include "omp.h"

#include "scene.h"
#include "integrator.h"

int main(int argc, char* argv[]) {
    omp_set_num_threads(4);
    simple_pt::Scene scene;
    simple_pt::Integrator integrator;
    if (argc >= 3) {
        scene.load(argv[1], argv[2], argv[3]);
        integrator.loadCamera(argv[2]);
        integrator.render(scene);
        integrator.draw();
    }
    return 0;
}