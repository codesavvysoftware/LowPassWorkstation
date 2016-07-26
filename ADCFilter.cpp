////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ADCFilter.cpp
///
/// Implementation of the ADCFilter class
///
/// @see ADCFilter.hpp for a detailed description of this class.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley1   09-May-2016 Original Implementation
/// @endif
///
/// @ingroup Low Pass Filters
///
/// @par Copyright (c) 2016 Rockwell Automation Technologies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES

// C PROJECT INCLUDES

// C++ PROJECT INCLUDES
#include "ADCFilter.hpp"

namespace LowPassFilters
{
    //**************************************************************************************************************
    // Public methods
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: ADCFilter::ApplyFilter
    ///
    /// Apply the low pass filter difference equation to unfiltered ADC input data
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void ADCFilter::ApplyFilter(int32_t  slADCValueRead, 
                                uint32_t  ulCornerFreqToFilterHZ,  
                                int32_t & rslFilterOutput,
                                bool &    rbFilterAppliedSuccessfully)
    {
        int32_t slScaledADCValue = slADCValueRead << 15;

        rslFilterOutput = slScaledADCValue;
            
        if (
                 !IsFilteringReadyToStart()
              || !ReconfigureWithNewCornerFrequency(ulCornerFreqToFilterHZ)
           )
        {
            rbFilterAppliedSuccessfully = false;

            return;
        }

        rbFilterAppliedSuccessfully = true;

        //
        // Filter restarting state is true when the filter has been configured and the first sample has not been applied or
        // when the ApplyFilter method yields an invalid filter result.  When the filter is restarting, the first ADC value
        // that is passed back to the caller from the ApplyFilter method is the output of the filter.  Therefore no 
        // calculations are necessary and thus the filter has been applied successfully.
        // An immediate return to the caller is taken since there is no calculation to do. 
        // The value passed to HasFilterRestarted is what the filter data is initialized with.  
        //
        if (HasFilterRestarted(rslFilterOutput))
        {
            return;
        }

        int32_t slCurrentFilterOutput = slScaledADCValue;

        for (uint32_t ul = 0;
                (ul < MAX_NUMBER_OF_POLES)
             && (ul < m_ulNumberOfPoles);
             ul++)
        {
            //
            // Using a 32 bit number, since the calculation can yield negative numbers a place for the sign
            // bit as to be allocated in a fixed point number.  Therefore the integer portion of a 32 bit fixed point
            // number includes the ADC value which is assumed to have a resolution 32 bits or less and a sign bit.
            // The number of fractional bits for a fixed point 32 bit number would be 32-1(for the sign bit)- ADC resolution.
            // In order to shift the ADC value to the integer portion of a 32 bit fixed point number it must be 
            // shifted left by the number of fractional bits. 
            //
            //  For a 32 bit number with 16 bits of ADC resolution it would look something like this.
            //
            //  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
            //  S  ----------------- ADC Value Read ---------- | ------------ Fractional Part --------------
            //  I
            //  G
            //  N
            // 
            //  B
            //  I
            //  T
            //
            //  y(n) = y(n-1) + ((ADCValueRead << (ADCResolution - 1)) - y(n-1)) >>  m_FrequencyShiftFactor
            //
            //  This function works for a finite set of corner frequencies
            //  100 HZ, 50 HZ, 25 HZ, 10 HZ, 5 HZ and 1 HZ
            // 
            //  As well as only one sample period of 50 microseconds.
            //
            // Regarding the filter.
            // Where:
            //   y(n) == Value when filter applied
            //   y(n-1) == Previous filter output.
            //   LagCoefficient = Constant value that will be discussed subsequently
            //   x(n) == Current ADC value to filter.
            //     
            //   LagCoefficient is the 
            //   (CornerFrequencyInRadiansPerSecond * SamplingPeriodInSeconds ) / (2 + (CornerFrequencyInRadiansPerSecond * SamplingPeriodInSeconds)
            //   LagCoefficient is derived from taking the following first order low pass filter S transform:
            //      1/(RCTimeConstant * s) and using the Tustin approximation for z which is
            //          z = (2/SamplingPeriod) * ((z-1)/(z+1))
            //
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
            //       Therefore
            //           LagCoefficient = LagConstCornerFreqSamplingPeriodProduct / (2.0 * LagConstCornerFreqSamplingPeriodProduct ) 
            // 
            // With a 50 microsecond sampling period and a corner frequency of 100 HZ the lag coefficient is:
            //   = ( 50 * 100 * LAG_COEFF_CONSTANT ) / ((50 * 50 * LAG_COEFF_CONSTANT) + 2)
            //   = 0.0155855546083
            //   
            //  Getting back to the fixed point integer with an ADC resolution of 16 bits.
            //
            //  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
            //  S  ----------------- ADC Value Read --------------| ------------ Fractional Part --------------
            //  I
            //  G
            //  N
            // 
            //  B
            //  I
            //  T
            //
            // the least significant bit of fractional part is 2**(-15)
            // to show this when bit 14 is the only bit set it represent the value of 1/2 or 2**-1
            // The hex value when bit 14 is the only bit set is 0x4000
            //  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
            //  -------------------- ADC Value Read ------------- | ------------ Fractional Part --------------
            //   0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  1  0  0  0  0  0  0  0  0  0  0  0  0  0  0
            //
            // Therefor .5 = 0x4000/X 
            //           X = 0x4000/0x8000
            //             = 16384/32768
            //             = 2**14 / 2**15
            //             = 2**14 * 2**(-15)
            //
            // the fixed point integer if converted to floating point, the fixed point number would be multiplied by 2**(-15)
            // the integer portion is multiplied by 2**15 
            // and shifting it right 6 places is dividing by 64 or 2**6.
            // Therefore shifting the integer portion right 6 places is equivalent to multiplying the integer portion by 2**(-21).
            // Now if 
            //    the result of the input ADC value was 1 (x(n))
            //    sampling with a 50 microsecond period
            //    corner frequency of 100 HZ
            //    previous filter output was 0 (y(n-1))
            // then the lag coefficient is 0.0155855546083 (see above)
            // Plugging these numbers back into the difference equation for the filter
            //     y(n) = y(n-1) + lag coefficient * (x(n)-y(n-1))
            //          = 0 +  0.0155855546083 * (1 - 0)
            //          =  0.0155855546083
            //
            // Using the ShiftFact to represent the shift factor
            //     x(n) = 1 as scaled it needs to be shifted 15 places (number of fractional bits) to be to the immediate left of the binary point.
            // Therefore
            //     y(n)                    = y(n-1) + (((x(n) << 15 ) - y(n-1) >> ShiftFact) * 2**(-15)
            //      0.0155855546083        = 0 + (((1 << 15) - 0 ) >> ShiftFact ) * 2**(-15)
            //      0.0155855546083        = (0x8000 >> ShiftFact)*2**(-15) 
            //      0.0155855546083        = (0x8000 / ( (2**ShiftFact) * 2**(-15))
            //      0.0155855546083        = (0x8000 * 2**(-15)) / 2**ShiftFact
            //      0.0155855546083        = 1 / 2**ShiftFact
            //      2**ShiftFact           = 1/ 0.0155855546083
            //      2**ShiftFact           = 64.16
            //      LogBase2(2**ShiftFact) = LogBase2(64.16)
            //      ShiftFact              = 6
            //
            // Further an assumption is made that the Lag Coefficient acroos the frequency range of 100 HZ to 1 HZ
            // is linear for a sample rate of 50 microseconds.
            // Thus if the difference between the current raw ADC value and the previous filter output is shifted
            // right 6 places for a corner frequency of 100 HZ, it is shift 7 places for a corner frequency of 50 HZ 
            // and 8 places for a corner frequency of 25 HZ.  In another products implementation 12.5 HZ was deemed
            // close enough to 10 HZ, 6.25 HZ was deemed close enough to 5 HZ and 1.5625 HZ was deemed close enough
            // to 1 HZ to just use shifting.
            //
            int32_t slLagTerm = slCurrentFilterOutput - m_slPole[ul];

            bool bInvalidFilterOutput = !IsFilterOutputValid(slLagTerm, slCurrentFilterOutput, m_slPole[ul]);

            if (bInvalidFilterOutput)
            {
                slCurrentFilterOutput = slScaledADCValue;

                rbFilterAppliedSuccessfully = false;

                break;
            }
                    
            slLagTerm >>= m_slFrequencyShiftFactor;

            //  y(n) = y(n-1) + ((slADCValueRead << NumberOfFractionalBits) - y(n-1)) >>  m_FrequencyShiftFactor
            slCurrentFilterOutput = m_slPole[ul] + slLagTerm;

            // Any addition or subtraction could cause an overflow condtion to be true.
            bInvalidFilterOutput = IsThereOverflowFromAddSbtrct(m_slPole[ul], slLagTerm, slCurrentFilterOutput);

            if (bInvalidFilterOutput)
            {
                slCurrentFilterOutput = slScaledADCValue;

                rbFilterAppliedSuccessfully = false;

                break;
            }

            // For multiple poles
            m_slPole[ul] = slCurrentFilterOutput;
        }

        rslFilterOutput = slCurrentFilterOutput;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: ADCFilter::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void ADCFilter::ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodUS, bool & rbFilterConfigured)
    {
        if (!IsADCResolutionInRange())
        {
            rbFilterConfigured = false;

        }
        else
        {
            rbFilterConfigured = true;

            //
            // Only certain corner frequencies are allowed for configuration
            //
            switch (ulCornerFreqHZ)
            {
            case FREQ_100_HZ:

                m_slFrequencyShiftFactor = SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ;

                break;

            case FREQ_50_HZ:

                m_slFrequencyShiftFactor = SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ + 1;

                break;

            case FREQ_25_HZ:

                m_slFrequencyShiftFactor = SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ + 2;

                rbFilterConfigured = true;

                break;

            case FREQ_10_HZ:

                m_slFrequencyShiftFactor = SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ + 3;

                break;

            case FREQ_5_HZ:

                m_slFrequencyShiftFactor = SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ + 4;

                break;

            case FREQ_1_HZ:

                m_slFrequencyShiftFactor = SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ + 6;

                break;

            default:

                rbFilterConfigured = false;

                break;
            }

            if (rbFilterConfigured)
            {
                SetCornerFreqHZ(ulCornerFreqHZ);

                SetFilteringConfigured();
            }
        }
    }
};