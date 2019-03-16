/*****************************************************************************
 * Project:  CAD_TestWork_Subtract_KinematicSolid_from_Parallelepiped
 * Purpose:  Test project
 * Author:   NikitaFeodonit, nfeodonit@yandex.com
 *****************************************************************************
 *   Copyright (c) 2019 NikitaFeodonit
 *
 *    This file is part of the
 *    CAD_TestWork_Subtract_KinematicSolid_from_Parallelepiped project.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published
 *    by the Free Software Foundation, either version 3 of the License,
 *    or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    See the GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program. If not, see <http://www.gnu.org/licenses/>.
 ****************************************************************************/

#include "TaskSolution.hpp"

#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace twm
{
Solid::~Solid() = default;
Sphere::~Sphere() = default;
Cylinder::~Cylinder() = default;
PointCloud::~PointCloud() = default;
KinematicSolid::~KinematicSolid() = default;

void KinematicSolid::fillPoints(const Solid& solid)
{
  //  2D -- start, {sizeY, sizeX}(of slice), {sizeX of src, step in X}
  //  3D -- z*sizeY*sizeX + y*sizeX + x, {sizeZ, sizeY, sizeX}(of slice), {sizeY*sizeX of src, sizeX of src, step in X}
  //unsigned long sHW = sUB.height() * sUB.width();
  //unsigned long sStart =
  //    sUB.minZ() * sHW + sUB.minY() * sUB.height() + sUB.minX();
  //unsigned long cHW = cUB.height() * cUB.width();
  //unsigned long cStart =
  //    cUB.minZ() * cHW + cUB.minY() * cUB.height() + cUB.minX();
  //std::gslice sphSlice(sStart, {sUB.depth(), sUB.height(), sUB.width()},
  //    {sHW, sUB.width(), 1});
  //std::gslice cylSlice(cStart, {cUB.depth(), cUB.height(), cUB.width()},
  //    {cHW, cUB.width(), 1});
  //auto sphCloud = mPoints.cloud()[sphSlice];
  //auto cylCloud = mPoints.cloud()[cylSlice];

  ULongBbox ulb = doubleToULongBbox(solid.bbox(), mPoints.gridDelta());
  unsigned long cloudSize = mPoints.cloud().size();
  unsigned long numYX = mPoints.numberY() * mPoints.numberX();

  for(unsigned long m = ulb.minZ(), maxZ = ulb.maxZ(); m <= maxZ; ++m) {
    for(unsigned long k = ulb.minY(), maxY = ulb.maxY(); k <= maxY; ++k) {
      for(unsigned long i = ulb.minX(), maxX = ulb.maxX(); i <= maxX; ++i) {
        unsigned long idx = m * numYX + k * mPoints.numberX() + i;
        if(idx >= cloudSize || mPoints.cloud()[idx] ^ !mFiller) {
          continue;
        }
        DoublePoint dPt = mPoints.ulongPointToCoordinate({i, k, m});
        bool val = !mFiller ^ solid.contains(dPt);
        mPoints.cloud()[idx] = val;
      }
    }
  }
}

void KinematicSolid::build()
{
  double t = mFunc.GetBeginParameter();
  double end = mFunc.GetEndParameter();

  while(t <= end) {
    double t2 = t + mDeltaT;
    if(t2 > end) {
      t2 = end;
    }

    DoublePoint pT = mFunc.Evaluate(t);
    DoublePoint pT2 = mFunc.Evaluate(t2);

    mSphere.setCenter(pT);
    fillPoints(mSphere);

    Cylinder cylinder(pT, pT2, mSphere.radius());
    fillPoints(cylinder);

    t += mDeltaT;
  }

  DoublePoint pEnd = mFunc.Evaluate(end);

  mSphere.setCenter(pEnd);
  fillPoints(mSphere);
}


// static
void FileWriter::write(
    const PointCloud& points, const std::string& skinFileName)
{
  std::ofstream out(skinFileName);
  out.precision(std::numeric_limits<double>::max_digits10);

  unsigned long cloudSize = points.cloud().size();
  unsigned long numYX = points.numberY() * points.numberX();

  for(unsigned long k = 0, sizeY = points.numberY(); k < sizeY; ++k) {
    for(unsigned long i = 0, sizeX = points.numberX(); i < sizeX; ++i) {
      unsigned long sizeZ = points.numberZ();
      unsigned long m = sizeZ;

      while(m > 0) {
        unsigned long idx = (m - 1) * numYX + k * sizeX + i;
        if(idx >= cloudSize || !points.cloud()[idx]) {
          --m;
          continue;
        }

        DoublePoint coord = points.ulongPointToCoordinate({i, k, m - 1});
        out << coord.x() << " " << coord.y() << " " << coord.z() << "\n";
//        --m;
        m = 0;
      }
    }
  }
}


}  // namespace twm
