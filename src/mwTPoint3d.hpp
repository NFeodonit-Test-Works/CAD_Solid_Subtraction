/******************************************************************************
*               File: mwTPoint3d.hpp                                          *
*******************************************************************************
*               Description:                                                  *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  26.06.2003 11:52:17 Created by: Sergej Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/

#ifndef __mwTPoint3d_hpp__
#define __mwTPoint3d_hpp__

#include <cmath>
#include <mwIdxRangeException.hpp>

#ifdef MW_5AXUTIL_DLL_VERSION
#include "5axutil_dllDefinition.hpp"
#else
#define MW_5AXUTIL_API
#endif

namespace cadcam
{
//!This struct represents a 3d point.
/*!The 3d point is an array with 3 dimension components (x,y,z).
  */
template <class T>
struct MW_5AXUTIL_API mwTPoint3d
{
public:
  //#############################################################################

  //!Sets 3d point to origin.
  inline mwTPoint3d()
  {
    elems[0] = 0;
    elems[1] = 0;
    elems[2] = 0;
  };

  //#############################################################################

  //!Set 3d point
  /*!Sets the 3d point to the new position.
      \param first constant x-coordinate of the 3d point
      \param second constant y-coordinate of the 3d point
      \param third constant z-coordinate of the 3d point
    */
  inline mwTPoint3d(const T& first, const T& second, const T& third)
  {
    elems[0] = first;
    elems[1] = second;
    elems[2] = third;
  };

  //#############################################################################

  //!Copy 3d point
  /*!Creates a exact copy of a given 3d point
       \param tc tool to create a copy of
    */
  inline mwTPoint3d(const mwTPoint3d& tc)
  {
    elems[0] = tc.elems[0];
    elems[1] = tc.elems[1];
    elems[2] = tc.elems[2];
  };

  //#############################################################################

  //!Set 3d point
  /*!Sets the 3d point to the new position.
      \param ar const array of the 3d point elements
    */
  inline mwTPoint3d(const T ar[])
  {
    elems[0] = ar[0];
    elems[1] = ar[1];
    elems[2] = ar[2];
  };

  //#############################################################################

  //!Assigment operator
  /*!Assigns one 3d point object to another
      \param tc tool to copy the information from
      \returns current 3d point with copied information
    */
  inline const mwTPoint3d& operator=(const mwTPoint3d& tc)
  {
    if(this != &tc) {
      elems[0] = tc.elems[0];
      elems[1] = tc.elems[1];
      elems[2] = tc.elems[2];
    }

    return *this;
  };

  //#############################################################################

  //!Assigment operator
  /*!Assigns one 3d point an array
      \param ar tool to copy the information from an array with 3d point information
      \returns current 3d point with copied information
    */
  inline const mwTPoint3d& operator=(const T ar[])
  {
    elems[0] = ar[0];
    elems[1] = ar[1];
    elems[2] = ar[2];

    return *this;
  };

  //#############################################################################

  //!Equality operator
  /*!Are given tools equal?
      \param tc tool to compare the current tool with
      \returns <b>true</b> if both tools contain the same parameters,
      <b>false</b> otherwise
    */
  inline bool operator==(const mwTPoint3d& tc)
  {
    return elems[0] == tc.elems[0] && elems[1] == tc.elems[1]
        && elems[2] == tc.elems[2];
  };

  //#############################################################################

  //!Inequality operator
  /*!Are given tools equal?
      \param tc tool to compare the current tool with
      \returns <b>false</b> if both tools contain the same parameters,
      <b>true</b> otherwise
    */
  inline bool operator!=(const mwTPoint3d& tc) { return !((*this) == tc); };

  //#############################################################################

  //!Set 3d point
  /*!Sets new x-coordinate of the 3d point.
      \param newx const x-coordinate of the 3d point
    */
  inline void x(const T& newx) { elems[0] = newx; };

  //#############################################################################

  //!Set 3d point
  /*!Sets new y-coordinate of the 3d point.
      \param newy const y-coordinate of the 3d point
    */
  inline void y(const T& newy) { elems[1] = newy; };

  //#############################################################################

  //!Set 3d point
  /*!Sets new z-coordinate of the 3d point.
      \param newz const z-coordinate of the 3d point
    */
  inline void z(const T& newz) { elems[2] = newz; };

  //#############################################################################

  //!Get x-coordinate
  /*!Gets the x-coordinate of the 3d point as a reference.
      \returns constant reference to the x-coordinate
    */
  inline const T& x() const { return elems[0]; };

  //#############################################################################

  //!Get y-coordinate
  /*!Gets the y-coordinate of the 3d point as a reference.
      \returns constant reference to the y-coordinate
    */
  inline const T& y() const { return elems[1]; };

  //#############################################################################

  //!Get z-coordinate
  /*!Gets the z-coordinate of the 3d point as a reference.
      \returns constant reference to the z-coordinate
    */
  inline const T& z() const { return elems[2]; };

  //#############################################################################

  //!Get 3d point
  /*!Gets a pointer to the 3d point array.
      \returns constant pointer to the 3d point array
    */
  inline operator const T*() const { return elems; };

  //#############################################################################

  //!Get 3d point
  /*!Gets a pointer to the 3d point array.
      \returns pointer to the 3d point array
    */
  inline operator T*() { return elems; };

  //#############################################################################

  //!Indexer operator for const values
  /*!Returns, for a given index, a refernce to a 3d point component.
      If index is greater than 2 an exception will be thrown.
      \param idx position of the 3d point array
      \returns reference to a 3d point component.
    */
  inline const T& operator[](const unsigned short idx)
  {
    if(idx > 2)
      throw misc::mwIdxRangeException(2, idx);
    return elems[idx];
  };

  //#############################################################################

  //!+= operator
  /*!Add each component of a 3d point to another
      \param ta const reference of a 3d point
    */
  inline void operator+=(const mwTPoint3d& ta)
  {
    elems[0] += ta.elems[0];
    elems[1] += ta.elems[1];
    elems[2] += ta.elems[2];
  };

  //#############################################################################

  //!-= operator
  /*!Subtract each component of a 3d point with another
      \param ta const reference of a 3d point
    */
  inline void operator-=(const mwTPoint3d& ta)
  {
    elems[0] -= ta.elems[0];
    elems[1] -= ta.elems[1];
    elems[2] -= ta.elems[2];
  };

  //#############################################################################

  //!~ operator
  /*!Returns length of the vektor represented by the 3d point
      \returns constant length of the vektor represented by the 3d point.
    */
  inline double operator~() const
  {
    return std::sqrt(
        elems[0] * elems[0] + elems[1] * elems[1] + elems[2] * elems[2]);
  };

  //#############################################################################

  //!*= operator
  /*!Multiplicate each 3d component with a factor
      \param scFactor const reference to a factor
    */
  inline void operator*=(const double& scFactor)
  {
    elems[0] *= scFactor;
    elems[1] *= scFactor;
    elems[2] *= scFactor;
  };

  //#############################################################################

  //!/= operator
  /*!Divide each 3d component with a factor
      \param scFactor const reference to a factor
    */
  inline void operator/=(const double& scFactor)
  {
    elems[0] /= scFactor;
    elems[1] /= scFactor;
    elems[2] /= scFactor;
  };

  //#############################################################################

  //!! operator
  /*!Normalized representation of the vector (3d point)
    */
  inline void operator!() { (*this) /= ~(*this); };

private:
  T elems[3];
};

//#############################################################################

//!+ operator
/*!Add each component of a 3d point to another
    \param first const reference of a 3d point
    \param second const reference of a 3d point
    \returns added const 3d point
  */
template <class T>
inline const mwTPoint3d<T> operator+(
    const mwTPoint3d<T>& first, const mwTPoint3d<T>& second)
{
  return mwTPoint3d<T>(
      first.x() + second.x(), first.y() + second.y(), first.z() + second.z());
};

//#############################################################################

//!- operator
/*!Subtract each 3d point component from another
    \param first const reference of a 3d point
    \param second const reference of a 3d point
    \returns subtracted const 3d point
  */
template <class T>
inline const mwTPoint3d<T> operator-(
    const mwTPoint3d<T>& first, const mwTPoint3d<T>& second)
{
  return mwTPoint3d<T>(
      first.x() - second.x(), first.y() - second.y(), first.z() - second.z());
};

//#############################################################################

//!* operator
/*!Multiplicate each 3d component with a factor.
    \param pt const reference to a 3d point
    \param factor const reference to a factor to multiplicate with
    \returns new 3d point object
  */
template <class T>
inline const mwTPoint3d<T> operator*(
    const mwTPoint3d<T>& pt, const double& factor)
{
  return mwTPoint3d<T>(pt.x() * factor, pt.y() * factor, pt.z() * factor);
};

//#############################################################################

//!* operator
/*!Multiplicate each 3d component with a factor.
    \param factor const reference to a factor to multiplicate with
    \param pt const reference to a 3d point
    \returns new 3d point object
  */
template <class T>
inline const mwTPoint3d<T> operator*(
    const double& factor, const mwTPoint3d<T>& pt)
{
  return mwTPoint3d<T>(pt.x() * factor, pt.y() * factor, pt.z() * factor);
};

//#############################################################################

//!/ operator
/*!Divide each 3d component with a factor.
    \param pt const reference to a 3d point
    \param factor const reference to a factor to divide with
    \returns new 3d point object
  */
template <class T>
inline const mwTPoint3d<T> operator/(
    const mwTPoint3d<T>& pt, const double& factor)
{
  return mwTPoint3d<T>(pt.x() / factor, pt.y() / factor, pt.z() / factor);
};


//#############################################################################

//!* operator
/*!Scalar product of to 3d points.
    \param first const reference to a 3d point
    \param second const reference to a 3d point
    \returns const scalar
  */
template <class T>
inline double operator*(const mwTPoint3d<T>& first, const mwTPoint3d<T>& second)
{
  return first.x() * second.x() + first.y() * second.y()
      + first.z() * second.z();
};

//#############################################################################

//!% operator
/*!Cross (vector) product of to 3d points
    \param first const reference to a 3d point
    \param second const reference to a 3d point
    \returns const 3d point
  */
template <class T>
inline const mwTPoint3d<T> operator%(
    const mwTPoint3d<T>& first, const mwTPoint3d<T>& second)
{
  mwTPoint3d<T> result;
  result.x(first.y() * second.z() - first.z() * second.y());
  result.y(first.z() * second.x() - first.x() * second.z());
  result.z(first.x() * second.y() - first.y() * second.x());
  return result;
};

//#############################################################################

//!! operator
/*!Normalized representation of the vector (3d point)
    \param vector const reference to a 3d point
    \returns const 3d point
  */
template <class T>
inline const mwTPoint3d<T> operator!(const mwTPoint3d<T>& vector)
{
  mwTPoint3d<T> result;
  result = vector / (~vector);
  return result;
};

  //template <class T > typedef struct mwTPoint3d				mwTVector3d;
#define mwTVector3d mwTPoint3d
//	typedef template< class T > struct mwTPoint3d<T> mwTVector3d<T>?
};  // namespace cadcam
#endif  //__mwTPoint3d_hpp__
