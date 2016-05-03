////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file OUVFilter.hpp
///
/// A derived class for the low pass filters from the LowPassFilterFixedPt class.                                      
/// Lag coefficient is a compile time constant, can't be reconifgured
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley 20-Apr-2016 Original implementation
/// @endif
///
/// @ingroup ???
///
/// @par Copyright (c) 2016 Rockwell Automation Technolgies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef OUV_FILTER_HPP
#define OUV_FILTER_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPassFiltersFixedPt.hpp"

namespace LowPassFilters
{
    class OUVFilter : public LowPassFilterFixedPt
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: OUVFilter::OUVFilter
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Constructor for filtering using fixed point arritmetic but is only configured at instantiation time
        ///
        /// @param  ulCornerFreqHZ       Initial corner frequency in herz
        /// @param  ulSamplingPeriodUS   Sampling period for the filter in microseconds
        /// @param  LagCoefficient       InitialLagCoefficient
        /// @param  AtoDResolutionBits   Bit resolution of ADC channels
        /// 
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        OUVFilter(uint32_t uiCornerFreqHZ,
                  uint32_t ulSamplingPeriodUS,
                  uint32_t LagCoeffecient,
                  uint32_t AtoDResolutionBits)
            : LowPassFilterFixedPt(uiCornerFreqHZ, ulSamplingPeriodUS, LagCoeffecient, AtoDResolutionBits)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: OUVFilter::~OUVFilter
        ///
        /// Destructor
        ///
        /// @par Full Description
        /// Destructor for filtering using fixed point arritmetic but is only configured at instantiation time
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~OUVFilter()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: OUVFilter::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Method for configuring the low pass filter that is just a return.
        ///
        /// @param  uiCornerFreqHZ         Corner Frequency for the filter in herz
        /// @param  uiSamplingPeriodUS     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ConfigureFilter(uint32_t uiCornerFreqHZ, unsigned uiSamplingPeriodUS)
        {
            return true;
        }

    private:

        //**************************************************************************************************************
        // Private methods and attributes
        //************************************************************************************************************** 
  
        // inhibit default constructor, copy constructor, and assignment
        OUVFilter();

        OUVFilter(OUVFilter &);
        
        OUVFilter & operator=(OUVFilter const&); // assign op. hidden
        
    };
}
#endif
