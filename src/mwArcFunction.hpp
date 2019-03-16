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

#ifndef __mwArcFunction_hpp__
#define __mwArcFunction_hpp__

#include "mwDiscreteFunction.hpp"

//!This class describes an arc with the sweep angle of 360� for the parameter
//!interval 0 - 1
class mwArcFunction : public mwDiscreteFunction
{
public:
  //!Default constructor of an abstract discrete function
  /*!Constructs a function object with given start and end t
    \param t1 start parameter for function evaluation
    \param t2 end parameter for function evaluation
    \param arcRad radius of the arc to be described thru the function
  */
  mwArcFunction(const double t1, const double t2, const double arcRad);

  //!Destructor
  ~mwArcFunction();

  //!Evaluate at a given t
  point3d Evaluate(const double t) const;

private:
  double mRadius;
};

#endif  //__mwArcFunction_hpp__
