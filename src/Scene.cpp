#include "Scene.h"

#include <GL/glut.h>
#include <iostream>

namespace asa
{
int Scene::testcase = Scene::TEST_ADVECTION;
bool Scene::pauseFlag = true;
uint Scene::nCellsX = 100;
uint Scene::nCellsY = 100;
float Scene::step = 0.01f;
float Scene::kDensity = 1.0f;
float Scene::kGravity = -1.0f;
float Scene::kViscosity = 0.001f;

Scene::Scene()
    : fluid(nullptr)
    , fluidViz(nullptr)
{
}

Scene::~Scene()
{
    if (fluidViz)
        delete fluidViz;
    if (fluid)
        delete fluid;
}

void Scene::printSettings()
{
    std::cerr << std::endl << "Current Settings:" << std::endl;
    std::cerr << "\t-gridsize " << nCellsX << " " << nCellsY << std::endl;
    std::cerr << "\t-step " << step << std::endl;
}

void Scene::init(int argc, char *argv[])
{
    // TODO: read program parameters
    int arg = 1;
    while (arg < argc) {
        if (!strcmp(argv[arg], "-gridsize")) {
            arg++;
        } else if (!strcmp(argv[arg], "-step")) {
            arg++;
        } else {
            std::cerr << std::endl << "Unrecognized option " << argv[arg] << std::endl;
            std::cerr << "Usage: practica3.exe -[option1] [settings] -[option2] "
                         "[settings] ..."
                      << std::endl;
            std::cerr << "Options:" << std::endl;
            std::cerr << "\t-test [advection|smoke]" << std::endl;
            std::cerr << "\t-gridsize [gridcells x] [gridcells y]" << std::endl;
            std::cerr << "\t-step [step size in secs]" << std::endl;
            break;
        }
    }

    init();
}

void Scene::init()
{
    printSettings();

    const Index2 gridSize(nCellsX, nCellsY);
    const AABox2 gridDomain(-2.0f, -2.0f, 2.0f, 2.0f);
    const Grid2 grid(gridDomain, gridSize);

    fluid = new Fluid2(grid);
    fluid->init();

    fluidViz = new FluidVisualizer2(*fluid);
    fluidViz->init();

    // initialize test
    initAnimation();
}

void Scene::initAnimation()
{
    switch (testcase) {
        case TEST_ADVECTION: {
            Array2<Vector3> &inkRGB = fluid->inkRGB;
            for (uint i = 2; i < inkRGB.getSize().x / 4; ++i)
                for (uint j = 2; j < inkRGB.getSize().y / 4; ++j)
                    inkRGB[Index2(i, j)] = Vector3(1, 1, 0);

            Array2<float> &u = fluid->velocityX;
            for (uint i = 0; i < u.getSize().x; ++i)
                for (uint j = 0; j < u.getSize().y; ++j)
                    u[Index2(i, j)] = 2.0f;

            Array2<float> &v = fluid->velocityY;
            for (uint i = 0; i < v.getSize().x; ++i)
                for (uint j = 0; j < v.getSize().y; ++j)
                    v[Index2(i, j)] = 2.0f;

            break;
        }
        case SMOKE: {
            break;
        }
    }
}

void Scene::pause()
{
    pauseFlag = !pauseFlag;
    std::cout << "Pause: " << (pauseFlag ? "ON" : "OFF") << std::endl;
}

void Scene::update()
{
    if (pauseFlag)
        return;

    animate();
}

void Scene::animate()
{
    fluid->advanceStep(step);
}

void Scene::display()
{
    fluidViz->draw();
}
}  // namespace asa
