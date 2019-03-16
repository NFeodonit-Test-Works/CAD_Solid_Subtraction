#include "CreateSkin.hpp"

#include "TaskSolution.hpp"

void CreateSkin(const cadcam::mwTPoint3d<double> refPoint,
    const unsigned long nx,
    const unsigned long ny,
    const unsigned long nz,
    const double sphereRad,
    mwDiscreteFunction& func,
    const double deltaT,
    const double delta,
    const std::string& skinFileName)
{
  twm::TaskSolution::createSkin(
      refPoint, nx, ny, nz, sphereRad, func, deltaT, delta, skinFileName);
}
