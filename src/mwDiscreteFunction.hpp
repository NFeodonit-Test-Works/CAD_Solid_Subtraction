/******************************************************************************
*               File: mwDiscreteFunction.hpp                                  *
*******************************************************************************
*               Description:                                                  *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  17.10.2003 08:34:07 Created by: Sergej Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/

#ifndef __mwDiscreteFunction_hpp__
#define __mwDiscreteFunction_hpp__

#include "mwException.hpp"
#include "mwTPoint3d.hpp"

class mwDiscreteFunction
{
public:
  typedef cadcam::mwTPoint3d<double> point3d;

  //!Default constructor of an abstract discrete function
  /*!Constructs a function object with given start and end t
    \param t1 start parameter for function evaluation
    \param t2 end parameter for function evaluation
  */
  inline mwDiscreteFunction(const double t1, const double t2)
      : mTBegin(t1)
      , mTEnd(t2){};

  //!Destructor
  inline virtual ~mwDiscreteFunction(){};

  //!Evaluate at a given t
  inline virtual point3d Evaluate(const double t) const
  {
    if(t < mTBegin || t > mTEnd)
      throw misc::mwException(0, "T is out of range");

    return point3d(0., 0., 0.);
  };

  //!Get the begin of the parameter interval
  inline double GetBeginParameter() const { return mTBegin; };

  //!Get the end of the parameter interval
  inline double GetEndParameter() const { return mTEnd; };

protected:
private:
  double mTBegin;
  double mTEnd;
};


#endif  //__mwDiscreteFunction_hpp__
