/******************************************************************************
*               File: mwIdxRangeException.cpp                                 *
*******************************************************************************
*               Description:                                                  *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  27.06.2003 09:17:00 Created by: Sergej Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/

#include <sstream>

#include "mwIdxRangeException.hpp"

namespace misc
{
//#############################################################################
mwIdxRangeException::mwIdxRangeException(
    const unsigned short maxIndex, const unsigned short givenIndex)
    : mwException(0, "Unknown exception")
    , mMaxIndex(maxIndex)
    , mGivenIndex(givenIndex)
{
  std::stringstream errorMsg;
  errorMsg << "Index out of bounds. Max index accepted: ";
  errorMsg << mMaxIndex;
  errorMsg << ", index given: ";
  errorMsg << mGivenIndex;
  errorMsg << ".";
  SetErrorMessage(errorMsg.str());
}

//#############################################################################
unsigned short mwIdxRangeException::GetMaxIndex() const
{
  return mMaxIndex;
}

//#############################################################################
unsigned short mwIdxRangeException::GetGivenIndex() const
{
  return mGivenIndex;
}
};  // namespace misc
