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

#ifndef TASKSOLUTION_HPP
#define TASKSOLUTION_HPP

#include <algorithm>
#include <cmath>
#include <utility>
#include <valarray>

#include "mwDiscreteFunction.hpp"
#include "mwTPoint3d.hpp"

namespace twm
{
template <typename T>
using Point = cadcam::mwTPoint3d<T>;

using ULong = unsigned long;
using DoublePoint = Point<double>;
using ULongPoint = Point<ULong>;


template <typename T>
class Bbox
{
public:
  Bbox() = default;
  Bbox(const Bbox<T>& other) = default;
  Bbox(Bbox<T>&&) = default;
  Bbox<T>& operator=(Bbox<T>&& other) = default;
  ~Bbox() = default;

  Bbox(const Point<T>& min, const Point<T>& max);

  Bbox<T>& operator=(const Bbox<T>& other);
  explicit operator bool() { return mIsValid; }

  const Point<T>& min() const { return mMin; }
  const Point<T>& max() const { return mMax; }

  T minX() const { return mMin.x(); }
  T minY() const { return mMin.y(); }
  T minZ() const { return mMin.z(); }

  T maxX() const { return mMax.x(); }
  T maxY() const { return mMax.y(); }
  T maxZ() const { return mMax.z(); }

  T width() const { return mMax.x() - mMin.x() + 1; }
  T height() const { return mMax.y() - mMin.y() + 1; }
  T depth() const { return mMax.y() - mMin.y() + 1; }

  bool intersects(const Bbox<T>& other) const;
  bool contains(const Bbox<T>& other) const;
  const Bbox<T>& merge(const Bbox<T>& other);
  const Bbox<T>& intersect(const Bbox<T>& other);

  void move(const Point<T>& delta);

private:
  Point<T> mMin;
  Point<T> mMax;
  bool mIsValid = false;
};

template <typename T>
inline Bbox<T>::Bbox(const Point<T>& min, const Point<T>& max)
    : mMin(min)
    , mMax(max)
    , mIsValid(true)
{
}

template <typename T>
inline Bbox<T>& Bbox<T>::operator=(const Bbox<T>& other)
{
  if(&other == this) {
    return *this;
  }

  mMin = other.mMin;
  mMax = other.mMax;
  return *this;
}

template <typename T>
inline bool Bbox<T>::intersects(const Bbox<T>& other) const
{
  return minX() <= other.maxX() && maxX() >= other.minX()
      && minY() <= other.maxY() && maxY() >= other.minY()
      && minZ() <= other.maxZ() && maxZ() >= other.minZ();
}

template <typename T>
inline bool Bbox<T>::contains(const Bbox<T>& other) const
{
  return minX() <= other.minX() && maxX() >= other.maxX()
      && minY() <= other.minY() && maxY() >= other.maxY()
      && minZ() <= other.minZ() && maxZ() >= other.maxZ();
}

template <typename T>
inline const Bbox<T>& Bbox<T>::merge(const Bbox<T>& other)
{
  if(mIsValid) {
    mMin = Point<T>(std::min(mMin.x(), other.mMin.x()),
        std::min(mMin.y(), other.mMin.y()), std::min(mMin.z(), other.mMin.z()));
    mMax = Point<T>(std::max(mMax.x(), other.mMax.x()),
        std::max(mMax.y(), other.mMax.y()), std::max(mMax.z(), other.mMax.z()));

  } else {
    mMin = other.mMin;
    mMax = other.mMax;
  }
  return *this;
}

template <typename T>
inline const Bbox<T>& Bbox<T>::intersect(const Bbox<T>& other)
{
  if(intersects(other)) {
    if(mIsValid) {
      mMin = Point<T>(std::max(mMin.x(), other.mMin.x()),
          std::max(mMin.y(), other.mMin.y()),
          std::max(mMin.z(), other.mMin.z()));
      mMax = Point<T>(std::min(mMax.x(), other.mMax.x()),
          std::min(mMax.y(), other.mMax.y()),
          std::min(mMax.z(), other.mMax.z()));
    } else {
      mMin = other.mMin;
      mMax = other.mMax;
    }

  } else {
    *this = Bbox<T>();
  }
  return *this;
}

template <typename T>
inline void Bbox<T>::move(const Point<T>& delta)
{
  mMin += delta;
  mMax += delta;
}

using DoubleBbox = Bbox<double>;
using ULongBbox = Bbox<ULong>;

inline ULongBbox doubleToULongBbox(const DoubleBbox& dbb, double gridDelta)
{
  DoublePoint min = dbb.min() / gridDelta;
  DoublePoint max = dbb.max() / gridDelta;

  ULongPoint umin(static_cast<ULong>(std::floor(min.x())),
      static_cast<ULong>(std::floor(min.y())),
      static_cast<ULong>(std::floor(min.z())));

  ULongPoint umax(static_cast<ULong>(std::ceil(max.x())),
      static_cast<ULong>(std::ceil(max.y())),
      static_cast<ULong>(std::ceil(max.z())));

  return {umin, umax};
}


class Solid
{
public:
  Solid() = default;

  Solid(const Solid&) = delete;
  Solid(Solid&&) = delete;
  Solid& operator=(const Solid&) = delete;
  Solid& operator=(Solid&&) = delete;

  virtual ~Solid();

  virtual const DoubleBbox& bbox() const = 0;
  virtual bool contains(const DoublePoint& point) const = 0;
};


class Sphere : public Solid
{
public:
  Sphere() = delete;
  Sphere(const Sphere&) = delete;
  Sphere(Sphere&&) = delete;
  Sphere& operator=(const Sphere&) = delete;
  Sphere& operator=(Sphere&&) = delete;

  Sphere(const DoublePoint& center, double radius);

  ~Sphere() override;

  const DoublePoint& center() const { return mCenter; }
  double radius() const { return mRadius; }
  void setCenter(const DoublePoint& center);

  // Solid interface
  const DoubleBbox& bbox() const override { return mBbox; }
  bool contains(const DoublePoint& point) const override;

private:
  DoubleBbox setBbox();

  DoublePoint mCenter;
  const double mRadius;
  DoubleBbox mBbox;
};

inline Sphere::Sphere(const DoublePoint& center, double radius)
    : mCenter(center)
    , mRadius(radius)
    , mBbox(setBbox())
{
}

inline void Sphere::setCenter(const DoublePoint& center)
{
  // Subtraction of the each component of the 3d point from the another 3d point.
  mBbox.move(center - mCenter);
  mCenter = center;
}


inline DoubleBbox Sphere::setBbox()
{
  DoublePoint r(mRadius, mRadius, mRadius);
  // Subtraction and addition of the each component
  // of the 3d point to the another 3d point.
  return {mCenter - r, mCenter + r};
}

inline bool Sphere::contains(const DoublePoint& point) const
{
  // Subtraction of the each component of the 3d point from the another 3d point.
  // Length of vector between two points less then mRadius.
  return ~(point - mCenter) <= mRadius;
}


class Cylinder : public Solid
{
public:
  Cylinder() = delete;
  Cylinder(const Cylinder&) = delete;
  Cylinder(Cylinder&&) = delete;
  Cylinder& operator=(const Cylinder&) = delete;
  Cylinder& operator=(Cylinder&&) = delete;

  Cylinder(const DoublePoint& pt1, const DoublePoint& pt2, double radius);

  ~Cylinder() override;

  const DoublePoint& pt1() const { return mPt1; }
  const DoublePoint& pt2() const { return mPt2; }

  // Solid interface
  const DoubleBbox& bbox() const override { return mBbox; }
  bool contains(const DoublePoint& point) const override;

private:
  DoubleBbox setBbox();

  const DoublePoint mPt1;
  const DoublePoint mPt2;
  const DoublePoint mVector;
  double mRadius;
  const double mRadiusMultVectorLength;
  const DoubleBbox mBbox;
};

inline Cylinder::Cylinder(
    const DoublePoint& pt1, const DoublePoint& pt2, double radius)
    : mPt1(pt1)
    , mPt2(pt2)
    , mVector(mPt2 - mPt1)
    , mRadius(radius)
    , mRadiusMultVectorLength(mRadius * ~mVector)  // ~ -- length of vector.
    , mBbox(setBbox())
{
}

inline DoubleBbox Cylinder::setBbox()
{
  // Set the inaccurate Bbox for Cylinder by two Spheres at the ends.
  DoubleBbox sb = Sphere(mPt1, mRadius).bbox();
  return sb.merge(Sphere(mPt2, mRadius).bbox());
}

inline bool Cylinder::contains(const DoublePoint& point) const
{
  // * -- dot product of vectors
  // % -- cross product of vectors
  // ~ -- vector length, |AB|
  //
  // Check if point lies between the planes of the two circular facets
  // of the cylinder and ...
  // (point - mPt1) * (mPt2 - mPt1) >= 0.0
  // &&
  // (point - mPt2) * (mPt2 - mPt1) <= 0.0
  // &&
  //
  // ... check if point lies inside the curved surface of the cylinder.
  // |(point - mPt1) % (mPt2 - mPt1)|
  // -------------------------------- <= mRadius
  //           |mPt2 - mPt1|
  //
  return ((point - mPt1) * mVector) >= 0.0 && ((point - mPt2) * mVector) <= 0.0
      && ~((point - mPt1) % mVector) <= mRadiusMultVectorLength;
}


class PointCloud : public Solid
{
public:
  PointCloud() = delete;
  PointCloud(const PointCloud&) = delete;
  PointCloud(PointCloud&&) = delete;
  PointCloud& operator=(const PointCloud&) = delete;
  PointCloud& operator=(PointCloud&&) = delete;

  PointCloud(const DoublePoint& referencePoint,
      ULong nx,
      ULong ny,
      ULong nz,
      double gridDelta);

  ~PointCloud() override;

  ULong numberX() const { return mNumberX; }
  ULong numberY() const { return mNumberY; }
  ULong numberZ() const { return mNumberZ; }
  double gridDelta() const { return mGridDelta; }

  std::valarray<bool>& cloud() { return mCloud; }
  const std::valarray<bool>& cloud() const { return mCloud; }

  DoublePoint ulongPointToCoordinate(const ULongPoint& ulongPt) const;

  // Solid interface
  const DoubleBbox& bbox() const override { return mBbox; }
  bool contains(const DoublePoint& point) const override;

private:
  DoubleBbox setBbox();

  // Point cloud reference point
  const DoublePoint mReferencePoint;

  // Number of points in x direction
  const ULong mNumberX;

  // Number of points in y direction
  const ULong mNumberY;

  // Number of points in z direction
  const ULong mNumberZ;

  // Distance between points in the point grid (same fo x, y and z directions)
  const double mGridDelta;

  const DoubleBbox mBbox;
  std::valarray<bool> mCloud;
};

inline PointCloud::PointCloud(const DoublePoint& referencePoint,
    ULong nx,
    ULong ny,
    ULong nz,
    double gridDelta)
    : mReferencePoint(referencePoint)
    , mNumberX(nx)
    , mNumberY(ny)
    , mNumberZ(nz)
    , mGridDelta(gridDelta)
    , mBbox(setBbox())
    // Fill PointCloud with undeleted ('true') points.
    , mCloud(true, mNumberX * mNumberY * mNumberZ)
{
}

inline DoublePoint PointCloud::ulongPointToCoordinate(
    const ULongPoint& ulongPt) const
{
  // * -- multiplicate each 3d component of point with mGridDelta constant.
  return DoublePoint(ulongPt.x(), ulongPt.y(), ulongPt.z()) * mGridDelta
      + mReferencePoint;
}


inline DoubleBbox PointCloud::setBbox()
{
  double maxX = mReferencePoint.x() + mNumberX * mGridDelta;
  double maxY = mReferencePoint.y() + mNumberY * mGridDelta;
  double maxZ = mReferencePoint.z() + mNumberZ * mGridDelta;
  return {mReferencePoint, {maxX, maxY, maxZ}};
}

bool inline PointCloud::contains(const DoublePoint& point) const
{
  return mBbox.contains(DoubleBbox(point, point));
}


class KinematicSolid : public Solid
{
public:
  KinematicSolid() = delete;
  KinematicSolid(const KinematicSolid&) = delete;
  KinematicSolid(KinematicSolid&&) = delete;
  KinematicSolid& operator=(const KinematicSolid&) = delete;
  KinematicSolid& operator=(KinematicSolid&&) = delete;

  KinematicSolid(PointCloud& points,
      Sphere& sphere,
      const mwDiscreteFunction& func,
      const double deltaT,
      bool filler);

  ~KinematicSolid() override;

  void build();

  // Solid interface, is not used.
  const DoubleBbox& bbox() const override { return mBbox; }
  bool contains(const DoublePoint&) const override { return false; }

private:
  void fillPoints(const Solid& solid);

  PointCloud& mPoints;
  Sphere& mSphere;
  const mwDiscreteFunction& mFunc;  // User's function for the Sphere motion.
  const double mDeltaT;
  bool mFiller;
  DoubleBbox mBbox;
};

inline KinematicSolid::KinematicSolid(PointCloud& points,
    Sphere& sphere,
    const mwDiscreteFunction& func,
    const double deltaT,
    bool filler)
    : mPoints(points)
    , mSphere(sphere)
    , mFunc(func)
    , mDeltaT(deltaT)
    , mFiller(filler)
{
}


class FileWriter
{
public:
  static void write(const PointCloud& cloud, const std::string& skinFileName);
};


class TaskSolution
{
public:
  static void createSkin(const DoublePoint& refPoint,
      const ULong nx,
      const ULong ny,
      const ULong nz,
      const double sphereRad,
      mwDiscreteFunction& func,
      const double deltaT,
      const double delta,
      const std::string& skinFileName);
};

// static
inline void TaskSolution::createSkin(const DoublePoint& refPoint,
    const ULong nx,
    const ULong ny,
    const ULong nz,
    const double sphereRad,
    mwDiscreteFunction& func,
    const double deltaT,
    const double delta,
    const std::string& skinFileName)
{
  // Fill the PointCloud with 'true' points in the given Bbox.
  twm::PointCloud points(refPoint, nx, ny, nz, delta);

  // Set the params of the Sphere.
  twm::Sphere sphere({0, 0, 0}, sphereRad);

  // Set the params of the KinematicSolid. Filler is 'false'.
  twm::KinematicSolid kinSolid(points, sphere, func, deltaT, false);

  // Fill the KinematicSolid with 'false' points
  // in the given PointCloud, with Sphere motion by user's function.
  kinSolid.build();

  // Write only the uppermost points from the resulting solid,
  // which have the largest value of the coordinates along the Z axis.
  twm::FileWriter::write(points, skinFileName);
}


}  // namespace twm
#endif  // TASKSOLUTION_HPP
