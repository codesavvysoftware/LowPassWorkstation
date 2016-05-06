////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ADCFilter.hpp
///
/// A derived class for the low pass filters from the LowPassFilterFixedPt class.                                      
/// Takes advantage of a 50 us sample period to implement multiplications by shifting
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley 03-May-2016 Original implementation
/// @endif
///
/// @ingroup NeoS Low Pass Filtering
///
/// @par Copyright (c) 2016 Rockwell Automation Technolgies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ADC_FILTER_HPP
#define ADC_FILTER_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPassFiltersFixedPt.hpp"

namespace LowPassFilters
{
    class ADCFilter : public LowPassFilterFixedPt
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::ADC Filter
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Constructor for filtering using the fast shift technique to multiply
        ///
        /// @pre    None
        /// @post   ADC Filter object instantiated for use in I/O connections.
        /// 
        /// @param  ulCornerFreqHZ         Initial corner frequency in herz
        /// @param  ulSamplingPeriodUS     Sampling period for the filter in microseconds
        /// @param  LagCoefficient         InitialLagCoefficient
        /// @param  ulADCResolutionBits    Bit resolution of ADC channels
        /// 
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ADCFilter(uint32_t ulCornerFreqHZ,
                  uint32_t ulSamplingPeriodUS,
                  uint32_t ulLagCoeffecient,
                  uint32_t ulADCResolutionBits)
            : m_slFrequencyShiftFactor(0),
            LowPassFilterFixedPt(ulCornerFreqHZ, ulSamplingPeriodUS, ulLagCoeffecient, ulADCResolutionBits, 1)// FIELD_SIDE_NUMBER_OF_POLES)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::~ADC Filter
        ///
        /// Destructor
        ///
        /// @pre    ADC Filter instantiated
        /// @post   ADC Filter deledted.
        /// 
        /// @par Full Description
        /// Destructor for filtering using the fast shift technique to multiply
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~ADCFilter()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Virtual method for the applying the low pass filter. Called synchronously at a periodic rate to compute a
        /// filtered output.  The output is calculated utilizing the following difference equation:
        /// y(n) = y(n-1) + Lag Coefficient * ( x(n) - y(n-1) )
        /// Where:
        ///     y(n)   = the output of the filter
        ///     y(n-1) = previous filter output
        ///     Lag Coefficient = (Corner Frequency in radians * Sample Period in seconds) / 2 + (Corner Frequency in radians * Sample Period in seconds)
        ///     x(n) = current ADC value read.
        ///  Regarding the lag coefficient
        ///
        ///    LagCoefficient is derived from taking the following first order low pass filter S transform:
        ///       1/(RCTimeConstant * s) and using the Tustin approximation for z which is
        ///       s = (2/SamplingPeriod) * ((z-1)/(z+1))
        ///
        /// @pre    ADC Filter instantiated
        /// @post   ADC Filter applied when the filter is enabled.
        /// 
        /// @param  slADCValueRead            Raw ADC data read by the ADC driver
        /// @param  ulCornerFreqToFilterHZ    Corner Frequency for the filter in herz.
        /// @param  rslFilterOutput           Output from filter difference equation
        ///
        /// @return  true when the filter was applied, 
        ///          false when the difference equation can't be applied.  Occurs when filter yields an invalid value,
        ///          the filter can produce an invalid value when addition of terms in the difference equation results
        ///          in an overflow. A return of false is also returned when the filter isn't ready to start.  The filter
        ///          isn't ready to start when the filter has not been configured with a corner frequency in herz that is in
        ///          one of the distinct values this class supports.  The class supports corner frequencies of 100 HZ, 50 HZ,
        ///          25 HZ, 10 HZ, 5 HZ, and 1 HZ only.  The sample period is 50 microseconds only. False is also returned 
        ///          when the filter can't be reconfigured. Calls to ApplyFilter() with a corner frequency that is different 
        ///          than the current corner frequency used will result in an attempt to configure the filter with the the 
        ///          input corner frequency in herz.  If the call to configure fails, reconfigure fails. 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ApplyFilter(int32_t slADCValueRead, uint32_t ulCornerFreqToFilterHZ,  int32_t & rslFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: ADCFilter::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Method for the configuring the low pass filter
        ///
        /// @pre    ADC Filter instantiated
        /// @post   Filter is configured according to valid sample period and corner frequencies. Invalid parameters 
        /// @post   inhibit configuration.
        /// 
        /// @param  ulCornerFreqHZ         Corner Frequency for the filter in herz
        /// @param  ulSamplingPeriodUS     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false when the corner frequency is not 100HZ, 50 HZ, 25HZ 10HZ, 5HZ or 1HZ.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodUS);

    private:

        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
       // Number of poles for field side MCU ADC channels
        static const uint32_t FIELD_SIDE_NUMBER_OF_POLES = 4;

        // Bit position where no shift is required for 100 HZ corner frequencies
        static const uint32_t SHIFT_FACTOR_FOR_100_HZ_CORNER_FREQ = 6;

        // Enumerated constant for corner frequencies that can be filtered for
        typedef enum
        {
            FREQ_100_HZ = 100,
            FREQ_50_HZ = 50,
            FREQ_25_HZ = 25,
            FREQ_10_HZ = 10,
            FREQ_5_HZ = 5,
            FREQ_1_HZ = 1
        }
        eVALID_FREQUENCIES;

        // Shift factor for multiplication by the lag coefficient
        // Determined at object instantiation time or during ADC 
        // ApplyFilter synchronous calls when the corner freq changes
        int32_t m_slFrequencyShiftFactor;

        // inhibit default constructor, copy constructor, and assignment
        ADCFilter();

        ADCFilter(ADCFilter &);

        ADCFilter & operator=(ADCFilter const&); // assign op. hidden
    };
};
#endif