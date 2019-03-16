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
  // These notes is about using of the std::valarray.
  //
  //  2D -- start, {sizeY, sizeX}(of slice), {sizeX of src, step in X}
  //  3D -- z*sizeY*sizeX + y*sizeX + x, {sizeZ, sizeY, sizeX}(of slice), {sizeY*sizeX of src, sizeX of src, step in X}
  //
  //ULong sHW = sUB.height() * sUB.width();
  //ULong sStart =
  //    sUB.minZ() * sHW + sUB.minY() * sUB.height() + sUB.minX();
  //
  //ULong cHW = cUB.height() * cUB.width();
  //ULong cStart =
  //    cUB.minZ() * cHW + cUB.minY() * cUB.height() + cUB.minX();
  //
  //std::gslice sphSlice(sStart, {sUB.depth(), sUB.height(), sUB.width()},
  //    {sHW, sUB.width(), 1});
  //std::gslice cylSlice(cStart, {cUB.depth(), cUB.height(), cUB.width()},
  //    {cHW, cUB.width(), 1});
  //
  //auto sphCloud = mPoints.cloud()[sphSlice];
  //auto cylCloud = mPoints.cloud()[cylSlice];


  // Check the points of the PointCloud only in the Bbox of the Solid
  // merged with the Bbox of the PointCloud.
  DoubleBbox dbb = solid.bbox();
  dbb.merge(mPoints.bbox());
  ULongBbox ulb = doubleToULongBbox(dbb, mPoints.gridDelta());

  ULong cloudSize = mPoints.cloud().size();
  ULong numYX = mPoints.numberY() * mPoints.numberX();

  // Set the points of the PointCloud, which belong to given Solid, to mFiller.
  for(ULong m = ulb.minZ(), maxZ = ulb.maxZ(); m <= maxZ; ++m) {
    for(ULong k = ulb.minY(), maxY = ulb.maxY(); k <= maxY; ++k) {
      for(ULong i = ulb.minX(), maxX = ulb.maxX(); i <= maxX; ++i) {
        // Get the point index from ULong coordinates.
        ULong idx = m * numYX + k * mPoints.numberX() + i;

        // Check the array boundaries.
        // Check if point of PointCloud is already filled.
        if(idx >= cloudSize || mPoints.cloud()[idx] ^ !mFiller) {
          continue;
        }

        // Get the normal point coordinates from ULong coordinates.
        DoublePoint dPt = mPoints.ulongPointToCoordinate({i, k, m});

        // Check if the point belongs to the solid. If it belongs, then fill
        // the point of PointCloud with mFiller, otherwise with !mFiller.
        bool val = !mFiller ^ solid.contains(dPt);
        mPoints.cloud()[idx] = val;
      }
    }
  }
}

void KinematicSolid::build()
{
  // Get 1st and last values of t from the user's function.
  double t = mFunc.GetBeginParameter();
  double end = mFunc.GetEndParameter();

  while(t <= end) {
    double t2 = t + mDeltaT;
    if(t2 > end) {
      t2 = end;
    }

    // Get next points from the user's function.
    DoublePoint pT = mFunc.Evaluate(t);
    DoublePoint pT2 = mFunc.Evaluate(t2);

    // Fill Sphere (at given point pT of center) with mFiller.
    mSphere.setCenter(pT);
    fillPoints(mSphere);

    // Fill Cylinder (with given radius and between given points pT and pT2)
    // with mFiller.
    Cylinder cylinder(pT, pT2, mSphere.radius());
    fillPoints(cylinder);

    t += mDeltaT;
  }

  // Get last point from the user's function.
  DoublePoint pEnd = mFunc.Evaluate(end);

  // Fill last Sphere (at given point pEnd of center) with mFiller.
  mSphere.setCenter(pEnd);
  fillPoints(mSphere);
}


// static
void FileWriter::write(
    const PointCloud& points, const std::string& skinFileName)
{
  // Write the points of the PointCloud to the file.
  std::ofstream out(skinFileName);
  out.precision(std::numeric_limits<double>::max_digits10);

  ULong cloudSize = points.cloud().size();
  ULong numYX = points.numberY() * points.numberX();

  for(ULong k = 0, sizeY = points.numberY(); k < sizeY; ++k) {
    for(ULong i = 0, sizeX = points.numberX(); i < sizeX; ++i) {
      ULong sizeZ = points.numberZ();
      ULong m = sizeZ;

      while(m > 0) {
        // Check the array boundaries.
        // Search 1st undeleted ('true') point, skip deleted ('false') points.
        ULong idx = (m - 1) * numYX + k * sizeX + i;
        if(idx >= cloudSize || !points.cloud()[idx]) {
          --m;
          continue;
        }

        // Write only the uppermost points from the resulting Subtracted Solid,
        // which have the largest value of the coordinates along the Z axis.
        DoublePoint coord = points.ulongPointToCoordinate({i, k, m - 1});
        out << coord.x() << " " << coord.y() << " " << coord.z() << "\n";
        m = 0;
//        --m;  // ... or all undeleted ('true') points.
      }
    }
  }
}


}  // namespace twm
