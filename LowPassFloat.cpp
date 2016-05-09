////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFloat.cpp
///
/// The base class for the low pass filters that use floating point arithmetic and derived from the template base class.
///
/// @see LowPassFloat.hpp for a detailed description of this class.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley 09-May-2016 Original implementation
/// @endif
///
/// @ingroup Low Pass Filters
///
/// @par Copyright (c) 2016 Rockwell Automation Technolgies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES

// C PROJECT INCLUDES
// (none)

// C++ PROJECT INCLUDES
#include "LowPassFloat.hpp"

namespace LowPassFilters
{
    //**************************************************************************************************************
    // Public methods
    //**************************************************************************************************************
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFloat::ApplyFilter
    ///
    /// Apply the low pass filter difference equation to unfiltered ADC input data
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LowPassFloat::ApplyFilter(float        fADCValueRead,
                                   uint32_t     ulCornerFreqToFilterHz,
                                   float &      rfFilterOutput,
                                   bool &       rbFilterAppliedSuccessfully)
    {
        rfFilterOutput = fADCValueRead;

        rbFilterAppliedSuccessfully = true;

        if ( 
                !IsFilteringReadyToStart() 
             || !ReconfigureWithNewCornerFrequency(ulCornerFreqToFilterHz)
           )
        {
            rbFilterAppliedSuccessfully = false;
        }
        //
        // Regarding HasFilterRestarted()
        // Filter restarting state is true when the filter has been configured and the first sample has not been applied or
        // when the ApplyFilter method yields an invalid filter result.  When the filter is restarting, the first A to D value
        // that is passed back to the caller from the ApplyFilter method is the output of the filter.  Therefore no 
        // calculations are necessary and thus the filter has been applied successfully.
        // An immediate return to the caller is taken since there is no calculation to do. 
        // The value passed to HasFilterRestarted is what the filter data is initialized with.  
        //
        else if (HasFilterRestarted(rfFilterOutput))
        {
            m_fPrevFilteredValue = fADCValueRead;

            rbFilterAppliedSuccessfully = true;
        }
        else
        {
            CalcDiffEquation(fADCValueRead, GetLagCoefficient(), rfFilterOutput, rbFilterAppliedSuccessfully);
        }

    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFilterFixedPt::CalcDiffEquation
    ///
    /// Calculate filtered output when applying the low pass filter
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LowPassFloat::CalcDiffEquation(float        fADCValueRead,
                                        float        fLagCoefficient,
                                        float &      rfFilteredOutput,
                                        bool  &      rbCalculationSuccess)
    {
        // Solve the lag filter equation besides adding to the previous filtered value and store it.
        float fDiff = (fADCValueRead - m_fPrevFilteredValue) * GetLagCoefficient();

        // Solve the lag filter equation described above.
        float fFilteredValue = m_fPrevFilteredValue + m_fRemainder + fDiff;

        // Check to see if the filter yielded a valid floating point number.  A
        // invalid floating point number is infinity or one when nan() returns true

        if (!IsFilterOutputValid(m_fPrevFilteredValue, fDiff, fFilteredValue))
        {
            RestartFiltering();

            rbCalculationSuccess = false;

            return;
        }
        if (    (fADCValueRead == 0.0f)
             && (fpclassify(fFilteredValue) == FP_SUBNORMAL)
             && (fFilteredValue != 0) 
           )
        {   
            // filter output is extremely near 0, headed to 0, but not 0.
            // round to 0   
            fFilteredValue = 0.0f;
        }
        
        //Check to make sure the diff was useful if not store it in the remainder until it becomes useful
        if (
                 (fFilteredValue == m_fPrevFilteredValue)
              && (fADCValueRead != fFilteredValue)
           )
        {
            m_fRemainder += fDiff;
        }
        else
        {
            m_fRemainder = 0.0f;
        }

        rfFilteredOutput = fFilteredValue;

        m_fPrevFilteredValue = fFilteredValue;

        rbCalculationSuccess = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: LowPassFloat::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void LowPassFloat::ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodUS, bool & rbFilterConfigured)
    {
        rbFilterConfigured = false;
        
        if (IsCornerFreqWithBounds(ulCornerFreqHZ)
            && IsSamplingPeriodWithBounds(ulSamplingPeriodUS)
            )
        {
            //
            // HZ represents hertz
            //
            SetCornerFreqHZ(ulCornerFreqHZ);

            //
            // US represents microseconds
            //
            SetSamplingPeriodUS(ulSamplingPeriodUS);

            // 
            // The difference equation implemented is 
            // y(n) = y(n-1) + LagCoefficient * (x(n) - y(n-1))
            // 
            // Where:
            //   y(n) == Value when filter applied
            //   y(n-1) == Previous filter output.
            //   LagCoefficient = Constant value that will be discussed subsequently
            //   x(n) == Current A to D value to filter.
            //     
            //   LagCoefficient is the 
            //   (CornerFrequencyInRadiansPerSecond * SamplingPeriodInSeconds ) / (2 + (CornerFrequencyInRadiansPerSecond * SamplingPeriodInSeconds)
            //   LagCoefficient is derived from taking the following first order low pass filter S transform:
            //      1/(RCTimeConstant * s) and using the Tustin approximation for z which is
            //          s = (2/SamplingPeriod) * ((z-1)/(z+1))
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
            //       An automatic float var is used fLagConstCornerFreqSamplingPeriodProduct.
            //       Therefore
            //           LagCoefficient = LagConstCornerFreqSamplingPeriodProduct / (2.0 * LagConstCornerFreqSamplingPeriodProduct ) 
            float fLagConstCornerFreqSamplingPeriodProduct = LAG_COEFF_CONSTANT * static_cast<float>(ulCornerFreqHZ) * static_cast<float>(ulSamplingPeriodUS);

            float fLagCoefficient = fLagConstCornerFreqSamplingPeriodProduct / (TWO_PT_ZERO + fLagConstCornerFreqSamplingPeriodProduct);

            SetLagCoefficient(fLagCoefficient);

            SetFilteringConfigured();

            rbFilterConfigured = true;
        }
    }
};
