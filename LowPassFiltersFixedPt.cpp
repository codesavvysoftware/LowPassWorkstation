////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFiltersFixedPt.cpp
///
/// Implementation of the LowPassFiltersFixedPt class
///
/// LowPass template class is instantiated with type int32_t.  The pure virtual methods needed to implemented with the following signatures
/// 
///        virtual bool ApplyFilter(int32_t adcValueRead, uint32_t ulCornerFreqHZ, int32_t & rFilterOutput);
///        virtual bool ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodHZ);
///        virtual bool CalcDiffEquation(int32_t   AtoDValue,
///                                      int32_t   LagCoefficient,
///                                      int32_t & FilteredValue);
///        virtual void InitFilterDataForRestart(int32_t InitialFilterOutput);
///        virtual bool IsFilterOutputValid(int32_t Term1,  int32_t Term2, int32_t Result);
///
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
             || !ReconfigureWithNewCornerFrequency(ulCornerFreqToFilter)
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
                (sizeof(m_slPole)>0)
             && ( ul < sizeof(m_slPole) / sizeof(uint32_t))
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

           slCurrentFilterResult = ScaledMultiply(slSecondTermOfDiffEq, slLagCoefficient, (GetNumberOfFractionalBits());

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

    int32_t LowPassFilterFixedPt::ScaledMultiply(int32_t slMultiplicand, int32_t slMultiplier, uint32_t ulNumberOfFractionalBits)
    {
        //
        //   Bit
        //   31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
        //   ----------------Integer Part-------------------|------------Fractional Part--------------------
        //
        // The product of two filxed point scaled numbers X * Y with Z fractional bits
        // X * Y =     ((X integer part >> Z ) * (Y integer Part >> Z )) << Z 
        //         +   (X fractional part) * (Y integer part)
        //         +   (X integer Part) * (Y fractional part)
        //         +   (X fractional part * (Y factional part)rounded to least significant bit of fractional part) >> (integer size - number of bits in fractional part)
        //
        //  An example with sixteen bits of fractional precision in a 32 bit integer
        //  X = 0x38000  //3.5 scaled  X integer part = 3    X fractional part = 0x8000
        //  Y = 0x28000  //2.5 scaled  Y integer part = 2    Y fractional part = 0x8000
        //  (X integer part >> Z) = 0x18000 >> 16 = 3;
        //  (Y integer part >> Z) = 0x28000 >> 16 = 2
        //  ((X integer part >> Z) * (Y integer part >> Z )) << Z = (3 * 2) << 16 = 0x60000;
        //  
        //  X fractional part * Y integer part = 0x8000 * 2 = 0x10000;
        //  X integer part * Y fractional pary = 3 * 0x8000 = 0x18000;
        //  
        // ( X fractional part * Y fractional part ) rounded >> (integer size - number of fracation bits)
        //    = (0x8000 * 0x8000) >> 16
        //    = (0x40000000) >> 16
        //    = 0x4000
        //   Adding 0x6000 + 0x1000 + 0x1800 + 0x4000 = 0x8C000 
        //   For 0x80400 the integer part is 8 and the fraction part is 0xC000 so with fraction precision of 16 bits it .75 since 0x8000 is 2**(-1) and 0x4000 is 2**(-2)
        //   So the product of the scaled numbers is 8.75 which is what we expect.
		//   
		//  One obvious question seemingly, why not just multiply X*Y IE 0x38000 * 0x2800?
		//  0x3800 * 0x1800 = 0x8C0000000 
		//  0x8C0000000 is more than 32 bits.  
		//  Doing that:
		//     X and Y have a fixed binary point with 16 bits of fractional precision.  Therefore they are scaled to 2**(-16).
		//     (X * 2**(-16)) * (Y * 2**(-16)) = (X * Y) * 2**(-32)
		//     Hopefully it easy to see that result of a simple multiply results in a number scaled to 2**(-32)
		//  
		//  A formuula for a scaled mulitply of X * Y
		//      X and Y have a fixed binary point with 16 bits of fractional precision. Therefore they are scaled to 2**(-16).
		//      (X * 2**(-16) * (Y * 2**(-16)) 
		//      = ((X>>16) * (Y>>16))<<16 + (X>>16) * (Y & 0xFFFF) + (Y>>16) * (X & 0xFFFF) + ((X & 0xFFFF) * (Y & 0xFFFF) + 0x8000) >> 16
		//
		//      X>>16 is retrieving the integer part of X
		//      Y>>16 is retrieving the integer part of Y
		//      (X & 0xFFFF) is retrieving the fractional part of X
		//      (Y & 0xFFFF) is retrieving the fractional part of Y
		//      = ((integer part of X) * (integer part of Y)<<16) + (integer part of X) * (fractional part of Y) + (integer part of Y) * (fractional part of X) + ((fractional part of X) * (fractional part of y)+0x8000)>>16
		// 
		// Explaining the term ((fractional part of X) * (fractional part of Y)+0x8000)>>16
		//     The fractional part of X * fractional part of Y
		//     = ((X & 0xFFFF) * 2**(-16)) * ((Y & 0xFFFF) * 2**(-16))
		//     = ((X & 0xFFFF) * (Y & 0xFFFF)) * 2**(-32)
		//     So multiplying the fractional parts results in a number scaled to 2**(-32).  The result nedes to be scaled to 2**(-16) so a shift to the right is needed.
		//     However before shifting a rounding of bit representing 2**(-17) is needed. 
		//     An example of why it is needed.  
		//     Say the result is 0x8001. Shifting it right 16 results in 0x00000000. So the 0x8001 bits (2**(-17)) to (2**(-32)) are lost.
		//     Adding 0x8000 to 0x8001 before shifting results in 0x00010001 then shifting 16 to the right results in 0x1.
		//     0x1 is closer to 0x8001 than 0x0 is to 0x8001 and thus the 2**(-17) to 2**(-32) bits have been rounded up.
        //   
        //   
		// What if the fractional precision varies? For example the number of bits of ADC resolution can vary?
		// For an ADC resolution of 16 bits the integer part of a scaled number needs to hold 17 bits, 16 bits for the ADC value and a sign bit.
		// Therefore the number of integer bits is 17 and the number of fractional bits is 15.
		// Also keep in mind that since the least significant bit of an integer quantity is bit 0 therefore the bit position to the right of the binary 
		// point is the number of fractional bits - 1.  So when shifting to the left by the number of fractional bits, a number is being shifted into
		// the integer portion of the fixed point number.



		// In the case of 16 bits of ADC resolution then 15 bits of fractional precision is required and the convention is that the scale factor is 15.
		// When the scale factor is 15 the binary point is at bit 14 IE bit 14 represent 2***(-1).
		// Whith 16 bits of fractional precision the scale factor is 16 and the binary point is at bit 15.
		// Rewriting the formula
        //    For the sake of simplicity
		//    ScaleFactor = ADC resoultion - 1.
		//    BinaryPt = ScaleFactor -1
		//    FractionMask = (1 << BinaryPt ) 
		//      (X * 2**(-16) * (Y * 2**(-16)) 
		//      = ((X>>(ScaleFactor -1)) * (Y>>(ScaleFactor-1)))<<16 + (X>>(ScaleFactor-1)) * (Y & 0xFFFF) + (Y>>16) * (X & 0xFFFF) + ((X & 0xFFFF) * (Y & 0xFFFF) + 0x8000) >> 16



        // The scale factor is the number of A/D resolution bits typically this routine applies to more than A/D filtering (just worked out that way)
        // The reason that the shift factor is decremented because the calculation is a signed calculation and thus
        // there is an extra bit allocated for the sign bit.
        uint32_t  ulBinaryPtBitPos = ulScaleFactor - 1;
		//
        //  For Multiplier * Multiplicand
		//
		// Integer part of the multiplier
        int32_t slMultiplierIntegerPart = RetrieveIntegerPartOfScaleddNmbrWithFixedBinaryPt(slMultiplier, ulBinaryPtBitPos);

        // Integer part of the multiplicand
        int32_t slMultiplicandIntegerPart = RetrieveIntegerPartOfScaleddNmbrWithFixedBinaryPt(slMultiplicand, ulBinaryPtBitPos);

        // Fractional part of the mulitiplier by masking the fractional part;
        uint32_t ulMultiplierFracPart = RetrieveFractionalPartOfScaleddNmbrWithFixedBinaryPt(slMultiplier, ulScaleFactor);

        // Fractional part of the mulitiplier by masking the fractional part;
        uint32_t ulMultiplicandFracPart = RetrieveFractionalPartOfScaleddNmbrWithFixedBinaryPt(slMultiplicand, ulScaleFactor);

        // Fractional part of the multiplicand * Fractional part of the multiplier rounded and then shifted to align with the binary point
        int32_t slProductOfFractionalParts = ComputeProductOfFracParts(ulMultiplicandFracPart, ulMultiplierFracPart, ulScaleFactor);

        // Integer part of the multiplier * Fractional part of the multiplicand no shifting required, binary alligned correctly
        int32_t slProductOfIntAndFracParts = ComputeProductOfIntegerPartAndFractionalPart(slMultiplierIntPart, ulMultiplicandFracPart);

        // Integer part of the multiplicand * fractional part of the multiplier shifted no shifting required, binary point aligned correctly.
        slProductOfIntAndFracParts += ComputeProductOfIntegerPartAndFractionalPart(slMultiplicandIntPart, ulMultiplierFracPart);

        // Integer part of the multiplicand * Integer part of the multiplier rounded and then shifted to align with the binary point
        int32_t slProductOfIntParts = ComputeProductOfIntegerParts(slMultiplicandIntPart, slMultiplierIntPart, ulBinaryPtBitPos);

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

