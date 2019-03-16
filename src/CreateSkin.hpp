#ifndef CREATESKIN_HPP
#define CREATESKIN_HPP

#include "mwDiscreteFunction.hpp"
#include "mwTPoint3d.hpp"

void CreateSkin(const cadcam::mwTPoint3d<double> refPoint,
    const unsigned long nx,
    const unsigned long ny,
    const unsigned long nz,
    const double sphereRad,
    mwDiscreteFunction& func,
    const double deltaT,
    const double delta,
    const std::string& skinFileName);

#endif /* CREATESKIN_HPP */
