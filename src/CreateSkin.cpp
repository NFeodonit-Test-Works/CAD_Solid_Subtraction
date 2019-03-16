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
  twm::PointCloud points(refPoint, nx, ny, nz, delta);
  twm::Sphere sphere({0, 0, 0}, sphereRad);
  twm::KinematicSolid kinSolid(points, sphere, func, deltaT, false);
  kinSolid.build();
  twm::FileWriter::write(points, skinFileName);
}
