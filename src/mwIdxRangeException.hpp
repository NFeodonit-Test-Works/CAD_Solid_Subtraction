/******************************************************************************
*               File: mwIdxRangeException.hpp                                 *
*******************************************************************************
*               Description:                                                  *
*                                                                             *
*******************************************************************************
*               History:                                                      *
*  27.06.2003 09:13:49 Created by: Sergej Nevstruyev                          *
*******************************************************************************
*               (C) 2003 by ModuleWorks GmbH                                  *
******************************************************************************/

#ifndef __mwIdxRangeException_hpp__
#define __mwIdxRangeException_hpp__

#include "mwException.hpp"

namespace misc
{
//!This class represents a index out of range exception.
class mwIdxRangeException : public mwException
{
public:
  //!Constructor
  /*!Constructs an exception object with given parameters:
      \param maxIndex const max index
      \param givenIndex const given index
    */
  mwIdxRangeException(
      const unsigned short maxIndex, const unsigned short givenIndex);

  //!The GetMaxIndex function
  /*!The GetMaxIndex function
      \return	const unsigned short
    */
  unsigned short GetMaxIndex() const;

  //!The GetGivenIndex function
  /*!The GetGivenIndex function
      \return	const unsigned short
    */
  unsigned short GetGivenIndex() const;

protected:
private:
  unsigned short mMaxIndex;
  unsigned short mGivenIndex;
};
};  // namespace misc
#endif  //__mwIdxRangeException_hpp__
