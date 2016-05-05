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
        
        int32_t slScaledAtoD = slAtoDValueRead << m_ulNumberOfFrcntlBits;
        
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

        //   The product of the CornerFrequencyInRadiansPerSecond * SamplingPeriodMicroSeconds is unitless
        //      CornerFrequencyInRadiansPerSecond == 2 * PI * CornerFreqHz
        //      the units are radians per second.
        //   The SamplingPeriodInSeconds = ulSamplingPeriodUs * .000001 seconds/per micosecond
        //   So to the following is used to compute the product of the CornerFequencyInRadiansPerSecond * SamplingPeriodInSeconds
        //        2 * PI * ulCornerFreqHz * ulSamplingPeriodUs * .000001
        //        Breaking this up (using associative law of multiplication) into a compile time constant multiplied by a run time value 
        //        we can state the following:
        //            Compile time constant == 2 * PI * .000001
        //            Run Time Value        == ulCornerFreqHz * ulSamplingPeriodUs
        //            if we call the compile constant = LAG_COEFF_CONSTANT then
        //            LagCoefficient = (LAG_COEFF_CONSTANT * ulCornerFreqHz * ulSamplingPeriodUs) / (2.0 + (LAG_COEFF_CONSTANT * ulCornerFreqHz * ulSamplingPeriodUs));
        //       In order to compute the following only once (LAG_COEFF_CONSTANT * ulCornerFreqHz * ulSamplingPeriodUs)
        //       An automatic float var is used fLagConstCornerFreqSamplingPeriodProduct.
        //       Therefore
        //           LagCoefficient = LagConstCornerFreqSamplingPeriodProduct / (2.0 * LagConstCornerFreqSamplingPeriodProduct ) 
        //
        //The lag coefficient :
        // 2?fT / ((2 + 2?fT))
        //   One of the implicit assumptions that was used in a previous implementation of a filter that I reviewed was that the fraction is linearly proportional 
        //   to the product of the corner frequency and sample period over a small range of values.The lag coefficient is in the form
        //    X / ((2 + X))
        //    I realized that if the above was linearly proportional to the product of the corner frequency and sample period then when using the first derivative 
        //    of a function in this form is even more so in being linearly proportional.The idea is to approximate the first derivative by a straight f(x) = mx + b 
        //    and integrate the function.The first derivative of the above functions is in the form :
        //        2 / ((2 + X)(2 + X))
        //    Easy to calculate b, when X = 0, b = 0.5;
        //    Without going through the entire calculation m is very close to - 0.5 over a range of corner frequency, sampling period 100 Hz and 500 us respectively.
        //  
        //    So the approximation of the first derivative of a straight line is :
        //    = 0.5 – 0.5x
        //
        //   Therefore the lag coefficient is approximated by integrating the straight line function :
        //   = 0.5x –(0.25)*(x**2)
        //
        //   When substituting 2?fT for  x, a multiply by .5 becomes a  shift right of one and a multiply by .25 becomes a shift to the right of 2 places.
        //   It is even better than that because .5 * 2?fT is ?fT and .25 * 2?fT squared is ?fT squared and thus the approximation can be written as
        //    ?fT*(1 - ?fT)
        //    ?fT =  (? * 0.000001) * ulCornerFreqHZ * ulSamplingPeriodUS
        //    LAG_CONSTANT = (? * 0.000001)
        //    LagCoefficient = LAG_CONSTANT * ulCornerFreqHZ * ulSamplePeriodUS * ( 1 - (LAG_CONSTANT * ulCornerFreqHZ * ulSamplePeriodUS))
        //   Since this is calculation is performed asynchronously and infrequently 64 bit values are used.  Basically this makes the code simpler
        //   LAG_CONSTANT = 13493;
        //   which as 64 bit integer with 32 bits of fractional precision is 13493 * 2**(-32) = 0.0000031415838 close enough to (? * 0.000001) 
        //   
        uint64_t ullAccum = LAG_CONSTANT * ulCornerFreqHZ * ulSamplePeriodUS;


        // (1-pi * corner_freq_hz * sample_period)
        uint64_t ullSecondTermInApprox = (ONE_PT_ZERO_SCALED_BINARY_PT - ullAccum);

        ullAccum *= ullSecondTermInApprox;

        ullAccum += ROUND_OFF_FRAC_64;

        // For th 64 bit value we want to shift the least significant bit for the 32 bit fixed point number to the
        // correct place in the 32 bit number.
        // We know result in llAccum is scaled to 2**(-64) since we multiplied two numbers scaled to 2**(-32)
        // Therefore shifting 32 bits to the right scales the number to 2**(-32) as the most significant bit represent 2**(-1) which is bit 31.
        // the 2**(-1) bit needs to be shifted to the correct place in the 32 bit fixed point number.  In order to shift it to the correct place
        // it must be shift one to the right of the least significant bit of the integer part.  Since the number of bits in the integer part is
        // the ADC resolution + 1, it needs to be shifted to the right by ADC resolution + 1.
        ullAccum >>= m_ulConfigFilterValForMovingFracBitsToFixedPtNumber;

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
        return !(     ((ulTerm1 & MOST_SIGNIFICANT_BIT_OF_UINT_VALS) ^ (ulTerm2 & MOST_SIGNIFICANT_BIT_OF_UINT_VALS)) 
                  &&  ((ulTerm1 & MOST_SIGNIFICANT_BIT_OF_UINT_VALS) ^ (ulResult & MOST_SIGNIFICANT_BIT_OF_UINT_VALS))
                );
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

           slCurrentFilterResult = ScaledMultiply(slSecondTermOfDiffEq, slLagCoefficient, GetNumberOfFractionalBits());

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
        //   ----------------Integer Part----------------|------------Fractional Part-----------------------
        //
        //  An example of a fixed point multiply with sixteen bits of fractional precision in a 32 bit integer
        //  X = 0x38000  //3.5 fixed point integer where the  X integer part = 3 and the X fractional part = 0x8000
        //  Y = 0x28000  //2.5 fixed point integer where the  Y integer part = 2 and the Y fractional part = 0x8000
        //  (X integer part >> 16) = 0x18000 >> 16 = 3;
        //  (Y integer part >> 16) = 0x28000 >> 16 = 2
        //  ((X integer part >> 16) * (Y integer part >> 16 )) << 16 = (3 * 2) << 16 = 0x60000;
        //  
        //  X fractional part * Y integer part = 0x8000 * 2 = 0x10000;
        //  X integer part * Y fractional pary = 3 * 0x8000 = 0x18000;
        //  
        // ( X fractional part * Y fractional part ) >> 16
        //    = (0x8000 * 0x8000) >> 16
        //    = (0x40000000) >> 16
        //    = 0x4000
        //   Adding 0x6000 + 0x1000 + 0x1800 + 0x4000 = 0x8C000 
        //   For 0x8C000 the integer part is 8 and the fraction part is 0xC000 so with fraction precision of 16 bits.
        //   Since 0xC000 represents .5 + .025 the fractional part of the product is .75 
        //   Since 0x80000 represents the integer part shifted 16 bits to the left, the integer part is 8
        //   So the product of the fixed point integers is 8.75 which is what we expect.
		//   
		//  One obvious question seemingly, why not just multiply X*Y IE 0x38000 * 0x2800?
		//  0x3800 * 0x1800 = 0x8C0000000   
		//  0x8C0000000 is more than 32 bits.  
		//  Doing that:
		//     X and Y have a fixed binary point with 16 bits of fractional precision.  Therefore they are scaled to 2**(-16).
		//     (X * 2**(-16)) * (Y * 2**(-16)) = (X * Y) * 2**(-32)
		//     Hopefully it easy to see that result of a simple multiply results in a number scaled to 2**(-32)
		//     By the way this might be ok if using 64 bit numbers provides better overall performance because the 32 bits values could
        //     be cast to 64 bit values, multiplied, and then shift right.  In the example above shifting right by 16 and the recasting to
        //     to a 32 bit value yields the correct results.
		//
        //
        //  A formuula for a scaled mulitply of X * Y
        //  X * Y =     ((X integer part >> Z ) * (Y integer Part >> Z )) << Z 
        //          +   (X fractional part) * (Y integer part)
        //          +   (X integer Part) * (Y fractional part)
        //          +   (X fractional part * (Y factional part) + (1 << (Z-1))) >> Z;
        //  Where Z is the number of fractional bits. 
        //
        //  Keep in mind that since the least significant bit of an integer quantity is bit 0 therefore the bit position to the right of the binary 
		//  point is the number of fractional bits - 1.  
        //
        //
        //  Extracting the integer parts and the fractional parts
        //      The integer parts need to be extracted such that the least significant bit of the integer part is at bit 0 which is accomplished by
        //      simply shifting to the right by the number of fractional bits
        //     
        //      The fractional parts need to be extracted such that the the integer part bits are masked off.  Doing an "AND" of the fixed point number
        //      with the appropriate bit mask is necessary.  Generating the bit mask can be accomplished by shifting a 1 to the left by the number of 
        //      fractional bits and then decrementing by 1.  
        //      if a 32 bit number has 16 fractional bits:
        //         Shifting 1 by 16 bits = 0x10000;
        //         Decrementing by 1 = 0xffff
        //      like wise if a 32 bit number has 15 fractional bits
        //         Shifting 1 by 15 bits = 0x8000
        //         Decrementing by 1 = 0x7fff
        //
        //
        //  Discussing the multiplication of the fractional parts, (X fractional part * (Y factional part) + (1 << (Z-1))) >> Z
        //      Say the X frational Part == 0x8002, respresenting 0x8001 * 2**(-16) or 1 * 2**(-1) + 1 * 2**(-15).
        //      and the Y fractional part == 1 and there are 16 bits of fractional precision.
        //      The product is obviously 0x8002 but since X and Y are scaled to 2**(-16), multiplying X and Y results in product scaled to 2**(-32)
        //      In order to scale the product to 2**(-16) the product must be shifted to the right by 16 bits.
        //      Shifting the product right by 16 bits results in value of 0.
        //      However, 0x8002 * 2**(-32) is actually closer to 1 * 2**(-16) than it is to 0 * 2**(-16)
        //      Therefore the product should be rounded up and then shifted.
        //      1 << (Z-1) when Z== 16 results in a value of 0x8000;
        //      Adding 0x8002 and 0x8000 result in 0x10002.
        //      shifting 0x10001 to right 16 places results in a value of 1.
        //
        //      To perhaps belabor the point say that the number of fractional bits is 15
        //      Say the X fractional part == 0x4001, representing 0x4001 * 2**(-15) or 1 * 2**(-1) + 1 * 2**(-15)
        //      and the Y fractiona part == 1 with 15 bits of fractional precesion.
        //      The product of the X fractional part and Y fractional part is 0x4001
        //      1 << (Z-1) where Z==15 result in a value of 0x4000
        //      Adding 0x4001 and 0x4000 results in 0x8001
        //      Shifting to the right by 15 places results in a value of 1 
        //      0x4001 * 2**(-30) is close to 1 * 2**(-15) than it is to 0.
        //
        //  Extracting the integer parts and the fractional parts
        //     Configure the bit mask for the fractional parts
        uint32_t ulFractionalPartBitMask = (1 << ulNumberOfFractionalBits) - 1;
        //
        //  Extracting the integer parts of X and Y
        int32_t slXIntegerPart = slMultiplicand >> ulNumberOfFractionalBits;
        int32_t slYIntegerPart = slMultiplier >> ulNumberOfFractionalBits;

        //  Extracting the fractional parts of X and Y
        int32_t slXFractionalPart = slMultiplicand & static_cast<int32_t>(ulFractionalPartBitMask);
        int32_t slYFractionalPart = slMultiplier & static_cast<int32_t>(ulFractionalPartBitMask);

        // Applying the formula
        //
        //    X * Y = ((X integer part >> Z) * (Y integer Part >> Z)) << Z
        //          +   (X fractional part) * (Y integer part)
        //          +   (X integer Part) * (Y fractional part)
        //          +   (X fractional part * (Y factional part) + (1 << (Z-1))) >> Z;
        //  where Z = ulNumberOfFractionalBtis
        //
        return(   (slXIntegerPart * slYIntegerPart) << static_cast<int32_t>(ulNumberOfFractionalBits)
                + (slXFractionalPart * slYIntegerPart)
                + (slXIntegerPart * slYFractionalPart)
                + (((slXFractionalPart * slYFractionalPart) + (static_cast<int32_t>(1 << ulFractionalPartBitMask))) >> ulFractionalPartBitMask));
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

