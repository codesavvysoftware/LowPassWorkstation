////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFloat.hpp
///
/// The base class for the low pass filters that use floating point arithmetic and derived from the template base class.
///
/// LowPass template class is instantiated with type float.  The pure virtual methods needed to implemented with the following signatures
/// 
///        virtual void ApplyFilter(float adcValueRead, uint32_t ulCornerFreqHZ, float & rFilterOutput, bool & rbFilterAppliedSuccessfully);
///        virtual void ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodHZ, bool & rbFilterConfigured);
///        virtual void CalcDiffEquation(float   ADCValue,
///                                      float   LagCoefficient,
///                                      float & rfFilteredValue,
///                                      bool &  rbCalculateSuccess);
///        virtual void InitFilterDataForRestart(float InitialFilterOutput);
///        virtual bool IsFilterOutputValid(float Term1,  float Term2, float Result);
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
#ifndef LOW_PASS_NEOS_FILTER_HPP
#define LOW_PASS_NEOS_FILTER_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPass.hpp"

namespace LowPassFilters
{
    static const float LAG_COEFF_CONSTANT = (2.0f * 3.1415927f * .000001f);

    static const float VAL_FOR_RESET_REMAINDER        = 0.0f;

    static const float TWO_PT_ZERO                    = 2.0f;

    class LowPassFloat : public LowPass<float>
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::LowPassFloat
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Base class contructor for the low pass filter using floating point arithmetic
        ///
        /// @pre    none
        /// @post   Object created.
        ///
        /// @param  ulCornerFreqHZ         Initial corner frequency in Herz
        /// @param  uiSamplingPeriodUS     Sampling period for the filter in microseconds
        /// @param  fLagCoefficient        InitialLagCoefficient
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
        LowPassFloat( uint32_t ulCornerFreqHZ,
                           uint32_t ulSamplingPeriodUS,
                           float    fLagCoefficient)
            : m_fPrevFilteredValue(0.0),
              m_fRemainder(0.0),
              LowPass(ulCornerFreqHZ, ulSamplingPeriodUS, fLagCoefficient)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::~LowPassFloat
        ///
        /// Destructor
        ///
        /// @par Full Description
        /// Base class Destructor for the low pass filter using floating point arithmetic
        ///
        /// @pre    Object previously created
        /// @post   Object destroyed
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~LowPassFloat()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::ApplyFilter
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
        /// @param  fADCValueRead                 Raw ADC data read by the ADC driver
        /// @param  ulCornerFreqToFilterHZ        Corner Frequency for the filter in herz.
        /// @param  rfFilterOutput                Output from filter difference equation
        /// @param  rbFilterAppliedSuccessfully   true when the filter was applied, 
        ///                                       false when the difference equation can't be applied.  Occurs when filter yields an invalid value,
        ///                                       the filter can produce an invalid value when the result of the difference equation results in a 
        ///                                       non floating point number or infinity. A return of false is also returned when the filter isn't 
        ///                                       ready to start.  The filter isn't ready to start when the filter has not been configured with a 
        ///                                       corner frequency in herz that is in the acceptable range.  The acceptable range of corner frequencies 
        ///                                       is determined at object instantiation time.  Also the filter isn't ready to start when it has not 
        ///                                       been configured with a sample period in microseconds that is in the acceptable range.  The accepatble 
        ///                                       range of sample periods is determined at object instantiation time.  False is also returned when the 
        ///                                       filter can't be reconfigured. Calls to ApplyFilter() with a corner frequency that is different than 
        ///                                       the current corner frequency used will result in an attempt to configure the filter with the saved 
        ///                                       sample period in microseconds and the the input corner frequency in herz.  If the call to configure 
        ///                                       fails, reconfigure fails. 
		///
		/// @return  none 
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void ApplyFilter(float        fADCValueRead, 
                         uint32_t     ulCornerFreqToFilterHZ, 
                         float &      rfFilterOutput,
			             bool &       rbFilterAppliedSuccessfully);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Virtual method for the configuring the low pass filter
        ///
        /// @pre    object created.
        /// @post   filter configure to input corner frequency and sample period.
        ///
        /// @param  ulCornerFreqHZ         Corner Frequency for the filter in Herz
        /// @param  ulSamplingPeriodUS     Sampling period for the filter in microseconds
		/// @param  rbFilterConfigured     true when the filter was configured, false when corner frequency or 
		///                                sampling period are not valid
        ///
        /// @return none 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodUS, bool & rbFilterConfigured); 

    protected:

        //**************************************************************************************************************
        // Protected methods and attributes
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::CalcDiffEquation
        ///
        /// Calculate filtered output when applying the low pass filter
        ///
        /// @par Full Description
        /// Virtual method for calculating filtered ADC value from the difference equation
        /// y(n) = y(n-1) + LagCoefficient * ( Raw_ADC_value_read - y(n-1) )
        /// where 
        ///   LagCoefficient
        ///    = (2 * pi * corner_frequenchy * sample_period ) / ( ( 2 * pi  * corner_frequenchy * sample_period ) + 2 )
        ///
        /// @pre    object created.
        /// @post   Difference equation calculated.
        ///
        /// @param  fADCValueRead            Raw ADC data scaled to a binary point
        /// @param  fLagCoefficient          Coefficient of the lag term.
        /// @param  rfFilteredValue          Value of raw ADC data filtered
		/// @param  rbCalculateSuccess       true when calculation occurred without error, 
		///                                  false when applying filter yields an invalid value or is not ready to start
		///
        /// @return  none
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void CalcDiffEquation(float        fADCValueRead,
                                      float        fLagCoefficient,
                                      float &      rfFilteredOutput,
			                          bool &       rbCalculateSuccess);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::InitFilterDataForRestart
        ///
        /// Initialize filter data to put the filter in it's initial state
        ///
        /// @par Full Description
        /// virtual method for initializing filter data.
        ///
        /// @pre    object created.
        /// @post   filter parameters for restarting the filter initialized.
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline virtual void InitFilterDataForRestart(float InitialFilterOutput)
        { m_fRemainder = VAL_FOR_RESET_REMAINDER; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::IsFilterResultValid
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Method for determining the validity of the low filter output generated by difference equation
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  DiffEqTerm1     First term of difference equation add
        /// @param  DiffEqTerm2     Second term of difference equation add
        /// @param  fFilterOutput   Result of difference equation add
        ///
        /// @return  true when the result is valid, false when one of the terms is an invalid float
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline virtual bool IsFilterOutputValid(float fDiffEqTerm1, float fDiffEqTerm2, float fFilterOutput)
        { return (IsFloatValid(fDiffEqTerm1) && IsFloatValid(fDiffEqTerm2) && IsFloatValid(fFilterOutput)); }


    private:
        
        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
        // Previous filter value required for difference equation
        float m_fPrevFilteredValue;

        // Used in calculating filter output
        float m_fRemainder;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFloat::IsFloatValid
        ///
        /// Determine validity of input float
        ///
        /// @par Full Description
        /// Method for determining the validity of a float
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  f     Float number to check
        ///
        /// @return  true when input float val is valid, false when float is not a number or infinity
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline bool IsFloatValid(float f)
        { return (!isnan(f) && !isinf(f)); }

        // inhibit default constructor, copy constructor, and assignment
        LowPassFloat();

        LowPassFloat(LowPassFloat &);

        LowPassFloat & operator=(LowPassFloat const&); // assign op. hidden
    };

};
#endif