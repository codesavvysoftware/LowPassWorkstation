////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFiltersFixedPt.cpp
///
/// Implementation of the LowPassFiltersFixedPt class
///
/// @see LowPassFiltersFixedPt.hpp for a detailed description of this class.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley1   03-May-2016 Original Implementation
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
    bool LowPassFilterFixedPt::ApplyFilter(int32_t slAtoDValueRead, uint32_t ulCornerFreqToFilter, int32_t & rslFilterOutput)
    {
        bool bSuccess = true;
        
        int32_t slScaledAtoD = slAtoDValueRead << m_ulScaledIntegerLSBBitPos;
        
        rslFilterOutput = slScaledAtoD;

        if (    !IsFilteringReadyToStart() 
             || !ReconfigureWithNewCornerFrequencey(ulCornerFreqToFilter)
           )
        {
            bSuccess = false;
        }
        else if (    !HasFilterRestarted(rslFilterOutput) 
                  && !CalcDiffEquation(slScaledAtoD, GetLagCoefficient(), rslFilterOutput)
                )
        {
            RestartFiltering();

            bSuccess = false;
        }
        
        return bSuccess;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplePeriodUS)
    {
        if (    !IsCornerFreqWithBounds(ulCornerFreqHZ)
             || !IsSamplingPeriodWithBounds(ulSamplePeriodUS)
           )
        {
            return false;
        }
            
        SetCornerFreqHZ(ulCornerFreqHZ);

        SetSamplingPeriodUS(ulSamplePeriodUS);

        // pi * corner_freq_hz * sample_period
        uint64_t ullAccum = PI_OMEGA * ulCornerFreqHZ * ulSamplePeriodUS;


        // (1-pi * corner_freq_hz * sample_period)
        uint64_t ullSecondTermInApprox = (ONE_PT_ZERO_SCALED_BINARY_PT - ullAccum);

        ullAccum *= ullSecondTermInApprox;

        ullAccum += ROUND_OFF_FRAC_64;

        ullAccum >>= (m_ulIntNumBitsInInt + (m_ulIntNumBitsInInt - m_ulNumberOfFrcntlBits));

        SetLagCoefficient(static_cast<uint32_t>(ullAccum));

        SetFilteringConfigured();

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
    void LowPassFilterFixedPt::InitFilterDataForRestart(int32_t slInitialFilterOutput)
    {            
        for (uint32_t ul = 0; 
                ( ul < sizeof(m_slPole) / sizeof(uint32_t))
             && ( ul < m_ulNumberOfPoles );
             ul++)
        {
            m_slPole[ul] = slInitialFilterOutput;
        }
    }                                                                         


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct(uint32_t ulTerm1, uint32_t ulTerm2, uint32_t ulResult)
    {
        bool bOverflowOccurred = false;

        // In pieces the following is implemented
        //
        // if (!((ulTerm1 & m_ulIntMSBSet) ^ (ulTerm2 & m_ulIntMSBSet) && ((ulTerm1 & m_ulIntMSBSet) ^ (ulResult & m_ulIntMSBSet))
        //
        // IF ( (the most significant bit of Terms one and 2 are the same) AND ( the most significant bits of term 1 and the result are different) )
        // then overflow occurred.
        //
        // m_ulIntMSBSet indicates a ul with the most significant bit set only.
        //
        uint32_t ulTerm1MSB = ulTerm1 & m_ulIntMSBSet;

        uint32_t ulTerm2MSB = ulTerm2 & m_ulIntMSBSet;

        uint32_t ulResultMSB = ulResult & m_ulIntMSBSet;

        if (!(ulTerm1MSB ^ ulTerm2MSB) && (ulTerm1MSB ^ ulResultMSB))
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
    bool LowPassFilterFixedPt::CalcDiffEquation(int32_t   slScaledAtoD,
                                                int32_t   slLagCoefficient,
                                                int32_t & rslFilteredValue)
    {
        uint32_t ul = 0;

        int32_t slCurrentFilterResult = slScaledAtoD;

        for (ul = 0; 
                ( ul <( sizeof(m_slPole) / sizeof(int32_t) ))
             && ( ul < m_ulNumberOfPoles );
             ul++)
        {
            int32_t slSecondTermOfDiffEq = slCurrentFilterResult - m_slPole[ul];

            bool bSubtractionOverflowed = IsThereOverflowFromAddSbtrct(slCurrentFilterResult, m_slPole[ul], slSecondTermOfDiffEq);

            if (bSubtractionOverflowed)
            {
                return false;
            }

           slCurrentFilterResult = ScaledMultiply(slSecondTermOfDiffEq, slLagCoefficient);

            int32_t slLagValue = slCurrentFilterResult;

            slCurrentFilterResult += m_slPole[ul];

            bool bAdditionOverflowed = IsThereOverflowFromAddSbtrct(slLagValue, m_slPole[ul], slCurrentFilterResult);

            if (bAdditionOverflowed)
            {
                return false;
            }

            m_slPole[ul] = slCurrentFilterResult;

        }

        rslFilteredValue = slCurrentFilterResult;

        return true;
    }

    int32_t LowPassFilterFixedPt::ScaledMultiply(int32_t slMultiplicand, int32_t slMultiplier, uint32_t ulScaleFactor)
    {
        //
        // multiplicand integer part * multiplier integer part
        //
        //   Scaled Fixed Point Number with 16 bits of fractional precsion
        //
        //   Bit
        //   31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
        //   ----------------Integer Part-------------------|------------Fractional Part--------------------
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
        //   Example of multiplying two scaled 32 bit integers with a binary point at bit 15 
        //     16 bits for the integer part and 16 bits for the fractional part.
        //     integer part of Multiplicand = (Multiplicand & 0xffff0000) >> 16;
        //     fractional part of Multiplicand =  Multiplicand & 0xffff;
        //     integer part of Multiplier = (Multiplier & 0xffff0000) >> 16;
        //     fractional part of Multipier = (Multiplier & 0xffff);
        //     Multiplicand * Multiplier 
        //          =    ( integer part of Multiplicand * integer part of Multiplier )
        //             + ( ( integer part of Multiplicand * fractional part of Multiplier * 2**(-16) )
        //             + ( integer part of Multiplier * fractional part of Multiplicand * 2**(-16) ) 
        //             + (fractional part of Multiplicand * fractional part of Multiplier * 2**(-32)
        //
        //     A little explanation.  Since were using scaled 32 integers with 16 bits of fractional values then the fractional part of 
        //     a scaled number represents the least significant 16 bits times 2**(-16).
        //     the integer part of the same number is the most signficant 16 bits time 2**0.
        //     Taking each part above
        //     the product of (integer part of Multiplicand * integer part of Multiplier) needs to be shifted left 16 bits.
        //     the product of ( integer part of Multiplicand * fractional part of Multiplier requires no shifing.
        //     the product of ( integer part of Multiplier * fractional part of Multiplicand requires no shifing.
        //     the product of ( fractional part of Multiplicand * fractional part of Multiplier requires rounding and then shifting.
        //         since the product is number that is scaled to 2**-32 and there are only 16 bits of fractional value,
        //         if 2**-17 is a one the number is rounded up by adding 1 * 2**(-17) IE 0x8000;
        //         After rounding the product is shifted right 16 places to align with the binary point.
        //
        // The scale factor is the number of A/D resolution bits typically this routine applies to more than A/D filtering (just worked out that way)
        // The reason that the shift factor is decremented because the calculation is a signed calculation and thus
        // there is an extra bit allocated for the sign bit.
        uint32_t  ulBinaryPtBitPos = ulScaleFactor - 1;

        // Integer part of the multiplier
        int32_t slMultiplierIntPart = TakeIntegerPartOfScaleddNmbrWithFixedBinaryPt(slMultiplier, ulBinaryPtBitPos);

        // Integer part of the multiplicand
        int32_t slMultiplicandIntPart = TakeIntegerPartOfScaleddNmbrWithFixedBinaryPt(slMultiplicand, ulBinaryPtBitPos);

        // Fractional part of the mulitiplier by masking the fractional part;
        uint32_t ulMultiplierFracPart = TakeFractionalPartOfScaleddNmbrWithFixedBinaryPt(slMultiplier, ulBinaryPtBitPos);

        // Fractional part of the mulitiplier by masking the fractional part;
        uint32_t ulMultiplicandFracPart = TakeFractionalPartOfScaleddNmbrWithFixedBinaryPt(slMultiplicand, ulBinaryPtBitPos);

        // Fractional part of the multiplicand * Fractional part of the multiplier rounded and then shifted to align with the binary point
        int32_t slProductOfFractionalParts = TakeProductOfFracParts(ulMultiplicandFracPart, ulMultiplierFracPart, ulBinaryPtBitPos);

        // Integer part of the multiplier * Fractional part of the multiplicand no shifting required, binary alligned correctly
        int32_t slProductOfIntAndFracParts = TakeProductOfIntPartAndFracPart(slMultiplierIntPart, ulMultiplicandFracPart);

        // Integer part of the multiplicand * fractional part of the multiplier shifted no shifting required, binary point aligned correctly.
        slProductOfIntAndFracParts += TakeProductOfIntPartAndFracPart(slMultiplicandIntPart, ulMultiplierFracPart);

        // Integer part of the multiplicand * Integer part of the multiplier rounded and then shifted to align with the binary point
        int32_t slProductOfIntParts = TakeProductOfIntParts(slMultiplicandIntPart, slMultiplierIntPart, ulBinaryPtBitPos);

        return (slProductOfIntParts + slProductOfIntAndFracParts + slProductOfFractionalParts);
   }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::IsFilterResultValid
    ///
    /// Determine validity of low pass filter output
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LowPassFilterFixedPt::IsFilterOutputValid(int32_t slDiffEqTerm1, int32_t slDiffEqTerm2, int32_t slFilterOuput)
    {
        bool bFilterOutputValid = !IsThereOverflowFromAddSbtrct(slDiffEqTerm1, slDiffEqTerm2, slFilterOuput);

        return bFilterOutputValid;
    }
};

