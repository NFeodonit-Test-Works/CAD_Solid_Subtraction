/******************************************************************************
*               File: mwArcFunction.hpp                                       *
*******************************************************************************
*               Description:                                                  *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  17.10.2003 08:48:13 Created by: Sergej Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/

#include "mwArcFunction.hpp"
#include "mwMathConstants.hpp"

//#############################################################################
mwArcFunction::mwArcFunction(
    const double t1, const double t2, const double arcRad)
    : mwDiscreteFunction(t1, t2)
    , mRadius(arcRad){};

//#############################################################################
mwArcFunction::~mwArcFunction() {}

//#############################################################################
mwArcFunction::point3d mwArcFunction::Evaluate(const double t) const
{
  (void) mwDiscreteFunction::Evaluate(t);

  point3d retValue;

  retValue.x(500. + mRadius * sin(cadcam::MW_2PI * t));
  retValue.y(250. + mRadius * cos(cadcam::MW_2PI * t));
  retValue.z(100.);

  return retValue;
}
