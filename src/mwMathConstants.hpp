/******************************************************************************
*               File: mwMathConstants.hpp                                     *
*******************************************************************************
*               Description:                                                  *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  07.07.2003 12:42:31 Created by: Sergej Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/

#ifndef __mwMathConstants_hpp__
#define __mwMathConstants_hpp__

#include <limits.h>
#include <math.h>

namespace cadcam
{
//!Math const PI
const double MW_PI = 3.1415926535897932384626433832795;

//!Math const 2*PI
const double MW_2PI = 2. * MW_PI;

//!Math const PI/2
const double MW_PI_2 = MW_PI / 2.;

//!Degrees to radians conversion factor
const double MW_D2R = MW_PI / 180.;

//Radian to degree conversion factor
const double MW_R2D = 180. / MW_PI;

//!Smallest difference between two values.
/*!In this case the values are considered equal.
  */
const double MW_MATH_TOL = 1e-12;

//!Max double
const double MW_MAX_DOUBLE = 1e308;
//!Min double
const double MW_MIN_DOUBLE = 2e-308;

//!Max float
const float MW_MAX_FLOAT = 3e38f;
//!Min float
const float MW_MIN_FLOAT = 1e-38f;

//!Max short
const short MW_MAX_SHORT = 32767;
//!Min short
const short MW_MIN_SHORT = -32768;

//!Max unsigned short
const unsigned short MW_MAX_USHORT = 0xffff;

//!Max int
const int MW_MAX_INT = 2147483647;
//!Min int
const int MW_MIN_INT = (-2147483647 - 1);

//!Max unsigned int
const unsigned int MW_MAX_UINT = 0xffffffff;

//!Max long
const long MW_MAX_LONG = 2147483647;
//!Min long
const long MW_MIN_LONG = (-2147483647 - 1);

//!Max unsigned long
const unsigned long MW_MAX_ULONG = 0xffffffffUL;

//!is equal ?
/*!Are given double values equal?
    \param val1 first value to compare
    \param val2 second value to compare
    \returns <b>true</b> if absolute difference between both values <=1e-12 (tolerance),
    <b>false</b> otherwise
  */
template <class T>
inline bool is_equal(const T& val1, const T& val2)
{
  return (fabs(val1 - val2) <= MW_MATH_TOL);
}

//!is equal ?
/*!Are given double values equal?
    \param val1 first value to compare
    \param val2 second value to compare
    \param prec precision
    \returns <b>true</b> if absolute difference between both values <=1e-12 (tolerance),
    <b>false</b> otherwise
  */
template <class T>
inline bool is_equal(const T& val1, const T& val2, const T& prec)
{
  return (fabs(val1 - val2) <= prec);
}

//!Get minimum
/*!Gets minimum for given parameters:
    \param v1 first value to compare
    \param v2 second value to compare
    \returns smaller one
  */
template <class T>
inline const T& mw_min(const T& v1, const T& v2)
{
  return v1 > v2 ? v2 : v1;
};

//!Get maximum
/*!Gets maximum for given parameters:
    \param v1 first value to compare
    \param v2 second value to compare
    \returns greater one
  */
template <class T>
inline const T& mw_max(const T& v1, const T& v2)
{
  return v1 > v2 ? v1 : v2;
};
};  // namespace cadcam

#endif  //__mwMathConstants_hpp__
