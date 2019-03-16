/******************************************************************************
*               File: mwException.cpp										  *
*******************************************************************************
*               Description: implementation of the exception class            *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  18.03.2003 11:37:34 Created by: Sergiy Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/
#include "mwException.hpp"

namespace misc
{
//#############################################################################
mwException::mwException(unsigned long newErrCode,
    const std::string& newErrMsg,
    const mwException* prevLevel)
    : defErrMsg(newErrMsg)
    , errCode(newErrCode)
{
  if(prevLevel) {
    unsigned long stackErrCode = prevLevel->GetErrorCode();
    std::string stackErrMsg = prevLevel->GetErrorMessage();
    exStack = prevLevel->exStack;
    exStack.insert(exStack.begin(),
        std::pair<unsigned long, std::string>(stackErrCode, stackErrMsg));
  }
}


//#############################################################################
mwException::mwException(unsigned long newErrCode,
    const char* newErrMsg,
    const mwException* prevLevel)
    : defErrMsg(newErrMsg)
    , errCode(newErrCode)
{
  if(prevLevel) {
    unsigned long stackErrCode = prevLevel->GetErrorCode();
    std::string stackErrMsg = prevLevel->GetErrorMessage();
    exStack = prevLevel->exStack;
    exStack.insert(exStack.begin(),
        std::pair<unsigned long, std::string>(stackErrCode, stackErrMsg));
  }
}

//#############################################################################
mwException::mwException(const mwException& toCopy)
    : defErrMsg(toCopy.defErrMsg)
    , errCode(toCopy.errCode)
    , exStack(toCopy.exStack)
{
}

//#############################################################################
const mwException& mwException::operator=(const mwException& toCopy)
{
  defErrMsg = toCopy.defErrMsg;
  errCode = toCopy.errCode;
  exStack = toCopy.exStack;

  return *this;
}

//#############################################################################
bool mwException::operator==(const mwException& toCompare) const
{
  if(defErrMsg == toCompare.defErrMsg && errCode == toCompare.errCode
      && exStack == toCompare.exStack)
    return true;

  return false;
}

//#############################################################################
bool mwException::operator!=(const mwException& toCompare) const
{
  return !(*this == toCompare);
}

//#############################################################################
const std::string& mwException::GetErrorMessage() const
{
  return defErrMsg;
}

//#############################################################################
long mwException::GetErrorCode() const
{
  return errCode;
}

//#############################################################################
const mwException::exceptionStack& mwException::GetExceptionStack() const
{
  return exStack;
}

//#############################################################################
void mwException::SetErrorMessage(const std::string& errorMessage)
{
  defErrMsg = errorMessage;
}

//#############################################################################
void mwException::SetErrorCode(const unsigned long newErrorCode)
{
  errCode = newErrorCode;
}
};  // namespace misc
