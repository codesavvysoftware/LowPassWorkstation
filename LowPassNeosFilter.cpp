////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassNeosFilter.cpp
///
/// The base class for the low pass filters that use floating point arithmetic and derived from the template base class.
///
/// @see LowPassNeosFilter.hpp for a detailed description of this class.
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

// SYSTEM INCLUDES

// C PROJECT INCLUDES
// (none)

// C++ PROJECT INCLUDES
#include "LowPassNeosFilter.hpp"

namespace LowPassFilters
{
    //**************************************************************************************************************
    // Public methods
    //**************************************************************************************************************
    
    /// FUNCTION NAME: LowPassNeosFilter::ApplyFilter
    ///
    /// Apply the low pass filter difference equation to unfiltered ADC input data
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassNeosFilter::ApplyFilter(float        fAtoDValueRead,
        uint32_t     uiCornerFreqToFilter,
        float &      rfFilterOutput)
    {
        rfFilterOutput = fAtoDValueRead;

        bool bSuccess = true

        if ((!IsFilteringEnabled() || !ReconfigureWithNewCornerFrequencey(ulCornerFreqToFilter))
        {
            bSuccess = false;
        }
        else if (HasFilterRestarted(rslFilterOutput))
        {
            m_fPrevFilteredValue = fAtoDValueRead;
        }
        else
        {
            bSuccess = CalcDiffEquation(fAtoDValueRead, GetLagCoefficient(), rfFilterOutput);
        }

        return bSuccess;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::CalcDiffEquation
    ///
    /// Calculate filtered output when applying the low pass filter
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassNeosFilter::CalcDiffEquation(float        fAtoDValueRead,
                                             float        fLagCoefficient,
                                             float &      rfFilteredOutput )
    {
        // Solve the lag filter equation besides adding to the previous filtered value and store it.
        float fDiff = (fAtoDValueRead - m_fPrevFilteredValue) * GetLagCoefficient();

        // Solve the lag filter equation described above.
        float fFilteredValue = m_fPrevFilteredValue + m_fRemainder + fDiff;

        // Check to make sure the value isn't subnormal and trying to get to 0, if so we want to help it by setting 0
        // Subnormal check include inf/nan/zero, we don't want that so check those out too
        bool bFilterOutpuNotValid = !IsFilterOutputValid( m_fPrevFilteredValue, fDiff, fFilteredValue );

        if (bFilterOutputNotValid)
        {
            RestartFiltering();

            return false;
        }
        
        if (fAtoDValueRead == 0.0f)
        {
            fFilteredValue = 0.0f;
        }

        //Check to make sure the diff was useful if not store it in the remainder until it becomes useful
        if (
                 (fFilteredValue == m_fPrevFilteredValue)
              && (fAtoDValueRead != fFilteredValue)
           )
        {
            m_fRemainder += fDiff;
        }
        else
        {
            m_fRemainder = 0.0f;
        }

        rfFilterOutput = fFilteredValue;

        m_fPrevFilteredValue = fFilteredValue;

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassNeosFilter::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassNeosFilter::ConfigureFilter(uint32_t uiCornerFreq, uint32_t uiSamplingPeriod)
    {
        SetCornerFreq(uiCornerFreq);

        SetSamplingPeriod(uiSamplingPeriod);

        float TwoPiOmegaF = static_cast<float>(uiCornerFreq);

        TwoPiOmegaF *= static_cast<float>(uiSamplingPeriod);

        TwoPiOmegaF *= TWO_PI_SECONDS_PER_MICROSECOND;
            
        float LagCoefficient = TwoPiOmegaF / (TWO_PT_ZERO + TwoPiOmegaF);

        SetLagCoefficient(LagCoefficient);

        return true;
    }

    //**************************************************************************************************************
    // Protected methods and attributes
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassNeosFilter::InitFilterDataForRestart
    ///
    /// Initialize filter data to put the filter in it's initial state
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LowPassNeosFilter::InitFilterDataForRestart(float InitialFilterOutput)
    {            
        m_fRemainder = VAL_FOR_RESET_REMAINDER;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassNeosFilter::IsFilterResultValid
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassNeosFilter::IsFilterOutputValid(float fDiffEqTerm1, float fDiffEqTerm2, float fFilterOutput)
    {
        return (IsFloatValid(fDiffEqTerm1) && IsFloatValid(fDiffEqTerm2) && IsFloatValid(fFilterOutput));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassNeosFilter::IsFloatValid
    ///
    /// Determine validity of input float
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassNeosFilter::IsFloatValid(float f)
    {
      return (!isnan(f) && !isinf(f));
    }
};
