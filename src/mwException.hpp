/******************************************************************************
*               File: mwException.hpp										  *
*******************************************************************************
*               Description: Definition of an exception class                 *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  18.03.2003 10:57:48 Created by: Sergiy Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/
#ifndef __MW_EXCEPTION_HPP__
#define __MW_EXCEPTION_HPP__

#include <string>
#include <vector>

#ifdef MW_5AXUTIL_DLL_VERSION
#include "5axutil_dllDefinition.hpp"
#else
#define MW_5AXUTIL_API
#endif

namespace misc
{
//!Common exception class
/*!This class provides common exception functionality
    with stack of exception history
  */
class MW_5AXUTIL_API mwException
{
  //#############################################################################
public:
  typedef std::vector<std::pair<unsigned long, std::string>> exceptionStack;

  //!Default constructor
  /*!\param errCode error code of the exception.
      The default value of 0 is an unknown error
      \param errMsg default error message (english) for the given error
      code
      \param prevLevel exception catched from the previous level.
      NULL if this is the first level of no exception history should be
      provided
    */
  explicit mwException(unsigned long errCode,
      const std::string& errMsg = "Unknown error.",
      const mwException* prevLevel = 0);

  //!Construct an exception object
  explicit mwException(unsigned long errCode,
      const char* errMsg = "Unknown error.",
      const mwException* prevLevel = 0);

  //!Copy constructor
  /*!Create a copy of the given exception
      \param toCopy exception to create a copy of
    */
  mwException(const mwException& toCopy);

  //!Assigment operator
  /*!Assign the given exception to the current one
      \param toCopy exception to create a copy of
    */
  const mwException& operator=(const mwException& toCopy);

  //!Equality operator
  /*!Are exceptions equal
      \param toCompare exception to compare the current exception with
      \returns true if exceptions are equal
    */
  bool operator==(const mwException& toCompare) const;

  //!Inequality operator
  /*!Are exceptions equal
      \param toCompare exception to compare the current exception with
      \returns true if exceptions are not equal
    */
  bool operator!=(const mwException& toCompare) const;

  //!Get error message
  /*!Get the default error message of this exception
      \returns reference to a string containing the error message
    */
  const std::string& GetErrorMessage() const;

  //!Get error code
  /*!Get error code of the exception. The error code can be e.g. used to
      provide a language dependent information about the error occured
      \returns error code
    */
  long GetErrorCode() const;

  //!Get exception stack
  /*!Returns the array of the exceptions occured on the lower level
      \returns array of exceptions
    */
  const exceptionStack& GetExceptionStack() const;

  //!Set error message
  /*!Set the default error message of this exception
      \param errorMessage the new error message to set
    */
  void SetErrorMessage(const std::string& errorMessage);

  //!Set error code
  /*!Set error code of the exception. The error code can be e.g. used to
      provide a language dependent information about the error occured
      \param newErrorCode the new error code to set
    */
  void SetErrorCode(const unsigned long newErrorCode);


  //#############################################################################
protected:
  //#############################################################################
private:
  std::string defErrMsg;
  unsigned long errCode;
  exceptionStack exStack;
};
};  // namespace misc

#endif
