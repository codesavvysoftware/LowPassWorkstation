////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFiltersFixedPt.cpp
///
/// Implementation of the LowPassFiltersFixedPt class
///
/// @see LowPassFiltersFixedPt.hpp for a detailed description of this class.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley1   22-Apr-2016 Original Implementation
/// @endif
///
/// @ingroup ???
///
/// @par Copyright (c) 2016 Rockwell Automation Technologies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES

// C PROJECT INCLUDES
// (none)

// C++ PROJECT INCLUDES
#include "LowPassFiltersFixedPt.hpp"
//using namespace LowPassFilters;

namespace LowPassFilters
{
   
    //**************************************************************************************************************
    // Public methods
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::ApplyFilter
    ///
    /// Apply the low pass filter difference equation to unfiltered ADC input data
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::ApplyFilter(int32_t iAtoDValueRead, uint32_t uiCornerFreqToFilter, int32_t & rFilterOutput)
    {
        int32_t iScaledAtoD = iAtoDValueRead << m_ScaledIntegerLSBBitPos;

        rFilterOutput = iScaledAtoD;

        bool bFilteringIsNotEnabled = !IsFilteringEnabled();//  IsFilteringEnabled();

        if (bFilteringIsNotEnabled)
        {
           return false;
        }

   
        bool bReconfigureFailure = !ReconfigureWithNewCornerFrequencey(uiCornerFreqToFilter);
                
        if (bReconfigureFailure)
        {
            return false;
        }

        bool bLowPassFilteringStarting = HasFilterRestarted(rFilterOutput);

        if (bLowPassFilteringStarting )
        {
           return true;
        }

        int32_t iLagCoefficient = GetLagCoefficient();

        bool bErrorsCalcOfDiffEquation = !CalcDiffEquation(iScaledAtoD, iLagCoefficient, rFilterOutput);

        if (bErrorsCalcOfDiffEquation)
        {
            RestartFiltering();

            rFilterOutput = iScaledAtoD;

            return false;
        }

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::ConfigureFilter(uint32_t uiCornerFreq, uint32_t uiSamplePeriod)
    {
        SetCornerFreq(uiCornerFreq);

        SetSamplingPeriod(uiSamplePeriod);

        // pi * corner_freq_hz * sample_period
        uint64_t llAccum = PI_OMEGA * uiCornerFreq * uiSamplePeriod;


        // (1-pi * corner_freq_hz * sample_period)
        uint64_t llSecondTermInApprox = (0x100000000 - llAccum);

        llAccum *= llSecondTermInApprox;

        llAccum += ROUND_OFF_FRAC_64;

        llAccum >>= (m_IntNumBitsInInt + (m_IntNumBitsInInt - m_NumberOfFrcntlBits));

        uint32_t uiLagCoefficient = llAccum;

        SetLagCoefficient(uiLagCoefficient);

        return true;
    }

    //**************************************************************************************************************
    // Protected methods and attributes
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::InitFilterDataForRestart
    ///
    /// Initialize filter data to put the filter in it's initial state
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LowPassFilterFixedPt::InitFilterDataForRestart(int32_t InitialFilterOutput)
    {            
        for (int32_t i = 0; 
                ( i < sizeof(m_pole) / sizeof(uint32_t))
             && ( i < m_NumberOfPoles );
             i++)
        {
            m_pole[i] = InitialFilterOutput;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::IsFilterResultValid
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::IsFilterOutputValid(int32_t iDiffEqTerm1, int32_t iDiffEqTerm2, int32_t iFilterOuput)
    {
        bool bFilterOutputValid = !IsThereOverflowFromAddSbtrct(iDiffEqTerm1, iDiffEqTerm2, iFilterOuput);

        return bFilterOutputValid;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct(uint32_t uiTerm1, uint32_t uiTerm2, uint32_t uiResult)
    {
        bool bOverflowOccurred = false;

        uint32_t uiTerm1MSB = uiTerm1 & m_IntMSBSet;

        uint32_t uiTerm2MSB = uiTerm2 & m_IntMSBSet;

        uint32_t uiResultMSB = uiResult & m_IntMSBSet;

        if (!(uiTerm1MSB ^ uiTerm2MSB) && (uiTerm1MSB ^ uiResultMSB))
        {
            bOverflowOccurred = true;
        }

        return bOverflowOccurred;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::CalcDiffEquation
    ///
    /// Calculate filtered output when applying the low pass filter
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::CalcDiffEquation(int32_t   iScaledAtoD,
                                                int32_t   iLagCoefficient,
                                                int32_t & rFilteredValue)
    {
        int32_t i = 0;

        int32_t iCurrentFilterResult = iScaledAtoD;

        for (i = 0; 
                ( i <( sizeof(m_pole) / sizeof(int) ))
             && ( i < m_NumberOfPoles );
             i++)
        {
            int32_t iSecondTermOfDiffEq = iCurrentFilterResult - m_pole[i];

            bool bSubtractionOverflowed = IsThereOverflowFromAddSbtrct(iCurrentFilterResult, m_pole[i], iSecondTermOfDiffEq);

            if (bSubtractionOverflowed)
            {
                return false;
            }

            iCurrentFilterResult = ScaledMultiply(iSecondTermOfDiffEq, iLagCoefficient);

            int32_t iLagValue = iCurrentFilterResult;

            iCurrentFilterResult += m_pole[i];

            bool bAdditionOverflowed = IsThereOverflowFromAddSbtrct(iLagValue, m_pole[i], iCurrentFilterResult);

            if (bAdditionOverflowed)
            {
                return false;
            }

            m_pole[i] = iCurrentFilterResult;

        }

        rFilteredValue = iCurrentFilterResult;

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfADCResolutionBits
    ///
    /// Get the resolution in bits of the ADC inputs
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint32_t LowPassFilterFixedPt::GetNumberOfADCResolutionBits()
    {
        return m_AtoDResolutionBits;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfBitsInInt
    ///
    /// Get the number of bits in an int
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint32_t LowPassFilterFixedPt::GetNumberOfBitsInInt()
    {
        return m_IntNumBitsInInt;
    }

    int32_t LowPassFilterFixedPt::ScaledMultiply(int32_t multiplicand, int32_t multiplier, uint32_t scaleFactor)
    {
        //
        // multiplicand integer part * multiplier integer part
        //
        //   Shift multiplier integer part right so LSB is at bit 0.  Save in mltplrIntPartShftd.
        //   Shift multiplicand integer part right so LSB is at bit 0. Save in mltplcndIntPartShftd.
        //   Mask off integer part of multiplier and save in mlplrFracPart;
        //   Mask off integer part of multiplicand and save in mltplcndFracPart.
        //
        //   result =  ( mltplrIntPartShftd * mltplcndIntPartShftd ) << (scaleFactor -1 );
        //   result += ( mltplrIntPartShftd * mlplcndFracPart);
        //   result += ( mltplcndIntPartShftd * mltplrFracPart);
        //   result += ( ( mlplcndFracPart * mltplrFracPart ) + ( 1 << (scalefactor - 1) ) ) >> (scaleFactor -1 );
        // 
        uint32_t  shiftFactor = scaleFactor - 1;

        int32_t mltplrIntPartShftd = multiplier >> shiftFactor;

        int32_t mltplcndIntPartShftd = multiplicand >> shiftFactor;

        uint32_t FracMask = 1 << shiftFactor;

        FracMask -= 1;

        uint32_t mltplrFracPart = multiplier & FracMask;

        uint32_t mltplcndFracPart = multiplicand & FracMask;

        uint32_t roundoff = 1 << shiftFactor;

        int32_t result = ((!mltplcndFracPart || !mltplrFracPart) ? 0 : (((mltplcndFracPart * mltplrFracPart) + roundoff) >> shiftFactor));
            
        result += ((!mltplrIntPartShftd || !mltplcndIntPartShftd) ? 0 : (mltplrIntPartShftd * mltplcndIntPartShftd) << shiftFactor);

        result += ((!mltplrIntPartShftd || !mltplcndFracPart) ? 0 : mltplcndIntPartShftd * mltplcndFracPart);

        result += ((!mltplcndIntPartShftd || !mltplrFracPart) ? 0 : mltplcndIntPartShftd * mltplrFracPart);

        return result;

    }
};

