/////////////////////////////////////////////////////////////////////////////
/// @file CommonDeclares.hpp
///
/// This header file contains common header declarations used across various modules.
///
///
/// @par Full Description
///
///
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// -
/// @endif
///
/// @ingroup MySubsystem
///
/// @par Copyright (c) 2016 Rockwell Automation Technologies, Inc.  All rights reserved.
/////////////////////////////////////////////////////////////////////////////

#if !defined(COMMON_DECLARES_HPP)
#define COMMON_DECLARES_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES
#include "cppunit/extensions/HelperMacros.h"
#include <sstream>
#include <cstring>
#include <limits>
#include <cmath>
//#include "UnitTestLog/UtLog.hpp"

// C++ PROJECT INCLUDES
#ifndef nullptr
#define nullptr 0x00000000
#endif

//floating point constants
const float SNaN                     = std::numeric_limits<float>::signaling_NaN();
const float QNaN                     = std::numeric_limits<float>::quiet_NaN();
const float POSITIVE_INFINITY        = std::numeric_limits<float>::infinity(); // +INF
const float NEGATIVE_INFINITY        = (-1.0 * POSITIVE_INFINITY);             // -INF
const float POSITIVE_ZERO            = (float)(+0.0);
const float NEGATIVE_ZERO            = (float)(-0.0);
const float POSITIVE_DENORM_LBV      = (float)(1.40129846E-45);   //Positive denormalized lower boundary value. LSB of mantissa is 1 all others 0.
const float POSITIVE_DENORM_INRANGE  = (float)(2.546636E-39);     //Positive denormalized in range value.
const float POSITIVE_DENORM_UBV      = (float)(1.1754942E-38);    //Positive denormalized upper boundary value. All mantissa bits are 1.
const float NEGATIVE_DENORM_LBV      = (float)(-1.40129846E-45);  //Negative denormalized lower boundary value. LSB of mantissa is 1 sign bit 1.
const float NEGATIVE_DENORM_INRANGE  = (float)(-1.1750559E-38);   //Negative denormalized in range value. Exponent bits=0, Mantessa bits=11111111111001111000111.
const float NEGATIVE_DENORM_UBV      = (float)(-1.1754942E-38);   //Negative denormalized upper boundary value. Sign Bit(S)=1, Exponent bits(E)=0, Mantissa bits(M)=11111111111111111111111.
const float POSITIVE_NORM_LBV        = (float)(1.17549435E-38);   //Positive Normalized lower boundary. E=00000001, M=00000000000000000000000.
const float POSITIVE_NORM_INRANGE1   = (float)(2.3509886E-38);    //Positive Normalized in range value. E=00000001, M=11111111111111111111111.
const float POSITIVE_NORM_INRANGE2   = (float)(1.7014118E38);     //Positive Normalized in range value. E=11111110, M=00000000000000000000000.
const float POSITIVE_NORM_UBV        = (float)(3.4028235E38);     //Positive Normalized upper boundary value. E=11111110, M=11111111111111111111111.
const float NEGATIVE_NORM_LBV        = (float)(-1.17549435E-38);  //Negative Normalized lower boundary value. S=1, E=00000001, M=00000000000000000000000.
const float NEGATIVE_NORM_INRANGE1   = (float)(-2.3509886E-38);   //Negative Normalized in range value. S=1, E=00000001, M=11111111111111111111111.
const float NEGATIVE_NORM_INRANGE2   = (float)(-1.7014118E38);    //Negative Normalized in range value. S=1, E=11111110, M=00000000000000000000000.
const float NEGATIVE_NORM_UBV        = (float)(-3.4028235E38);    //Negative Normalized upper boundary value. S=1, E=11111110, M=11111111111111111111111.


#define ADD_TEST_METHOD(condition,pSuiteName,fixtureName,method,methodDescr)    \
{                                                                               \
  if(condition)                                                                 \
  {                                                                             \
      pSuiteName->addTest(new CppUnit::TestCaller<fixtureName>(methodDescr,&method));\
  }                                                                             \
}

//Function name to string
#define FUT(functionName)  #functionName

//extern UtLogInfo f_logObj;

#define TEST_ASSERT_MESSAGE(_x_,_message)                               \
if(!_x_ )                                                               \
{                                                                       \
    /*create cpp assertion message*/                                    \
    std::ostringstream strMsg("");                                      \
                                                                        \
    if(_message != NULL)                                                \
    {                                                                   \
        strMsg << _message << ". Actual value not equal to expected value."; \
    }                                                                   \
    else                                                                \
    {                                                                   \
        strMsg << "Actual value not equal to expected value.";          \
    }                                                                   \
                                                                        \
    const std::string& assertMsg = strMsg.str();                        \
                                                                        \
    const char* errorMsg = assertMsg.c_str();                           \
                                                                        \
    /*Perform Cpp Assertion*/                                           \
    CPPUNIT_ASSERT_MESSAGE(errorMsg, 0);                                \
}

#define TEST_ASSERT_EQUAL_MESSAGE(_actual_,_expected_,_message)         \
if( _actual_ != _expected_ )                                            \
{                                                                       \
    /*create cpp assertion message*/                                    \
    std::ostringstream strMsg("");                                      \
                                                                        \
    if(_message != NULL)                                                \
    {                                                                   \
        strMsg << _message << ". Actual value " << _actual_             \
        << " is not equal to expected value " << _expected_ << ".";     \
    }                                                                   \
    else                                                                \
    {                                                                   \
        strMsg << "Actual value " << _actual_                           \
        << " is not equal to expected value " << _expected_ << ".";     \
    }                                                                   \
                                                                        \
    const std::string& assertMsg = strMsg.str();                        \
                                                                        \
    const char* errorMsg = assertMsg.c_str();                           \
                                                                        \
    /*Perform Cpp Assertion*/                                           \
    CPPUNIT_ASSERT_MESSAGE(errorMsg, 0);                                \
}

#define TEST_ASSERT_NOT_EQUAL_MESSAGE(_actual_,_expected_,_message)         \
if( _actual_ != _expected_ )                                            \
{                                                                       \
    /*create cpp assertion message*/                                    \
    std::ostringstream strMsg("");                                      \
                                                                        \
    if(_message != NULL)                                                \
    {                                                                   \
        strMsg << _message << ". Actual value " << _actual_             \
        << " is not equal to expected value " << _expected_ << ".";     \
    }                                                                   \
    else                                                                \
    {                                                                   \
        strMsg << "Actual value " << _actual_                           \
        << " is not equal to expected value " << _expected_ << ".";     \
    }                                                                   \
                                                                        \
    const std::string& assertMsg = strMsg.str();                        \
                                                                        \
    const char* errorMsg = assertMsg.c_str();                           \
                                                                        \
    /*Perform Cpp Assertion*/                                           \
    CPPUNIT_ASSERT_MESSAGE(errorMsg, 0);                                \
}

#define TEST_ASSERT(_x_)                            TEST_ASSERT_MESSAGE(_x_,NULL)

#define TEST_ASSERT_EQUAL(_actual_,_expected_)      TEST_ASSERT_EQUAL_MESSAGE(_actual_,_expected_,NULL)

#define TEST_ASSERT_NOT_EQUAL(_actual_,_expected_)  TEST_ASSERT_NOT_EQUAL_MESSAGE(_actual_,_expected_,NULL)

#endif //#if !defined(COMMON_DECLARES_HPP)