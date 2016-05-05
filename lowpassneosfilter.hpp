////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassNeosFilter.hpp
///
/// The base class for the low pass filters that use floating point arithmetic and derived from the template base class.
///
/// LowPass template class is instantiated with type float.  The pure virtual methods needed to implemented with the following signatures
/// 
///        virtual bool ApplyFilter(float adcValueRead, uint32_t ulCornerFreqHZ, float & rFilterOutput);
///        virtual bool ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodHZ);
///        virtual bool CalcDiffEquation(float   AtoDValue,
///                                      float   LagCoefficient,
///                                      float & FilteredValue);
///        virtual void InitFilterDataForRestart(float InitialFilterOutput);
///        virtual bool IsFilterOutputValid(float Term1,  float Term2, float Result);
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

    class LowPassNeosFilter : public LowPass<float>
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::LowPassNeosFilter
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
 
        LowPassNeosFilter( uint32_t ulCornerFreqHZ,
                           uint32_t ulSamplingPeriodUS,
                           float    fLagCoefficient)
            : m_fPrevFilteredValue(0.0),
              m_fRemainder(0.0),
              LowPass(ulCornerFreqHZ, ulSamplingPeriodUS, fLagCoefficient)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::~LowPassNeosFilter
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
        virtual ~LowPassNeosFilter()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Virtual method for the applying the low pass filter
        ///
        /// @pre    ADC Filter instantiated
        /// @post   ADC Filter applied when the filter is enabled.
        /// 
        /// @param  fAtoDValueRead             Raw ADC data read by the ADC driver
        /// @param  ulCornerFreqToFilterHZ     Corner Frequency for the filter in herz.
        /// @param  rfFilterOutput             Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false when filter isn't ready to start or can't be reconfigured
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ApplyFilter(float        fAtoDValueRead, 
                         uint32_t     ulCornerFreqToFilterHZ, 
                         float &      rfFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::ConfigureFilter
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
        ///
        /// @return  true when the filter was configured, false when corner frequency or sampling period are not valid
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodUS); 

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
        /// @param  fAtoDValueRead           Raw ADC data scaled to a binary point
        /// @param  fLagCoefficient          Coefficient of the lag term.
        /// @param  rfFilteredValue          Value of raw ADC data filtered
        ///
        /// @return  true when calculation occurred without error, 
		///          false when applying filter yields an invalid value or is not ready to start
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool CalcDiffEquation(float        fAtoDValueRead,
                                      float        fLagCoefficient,
                                      float &      rfFilteredOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassNeosFilter::InitFilterDataForRestart
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
        /// FUNCTION NAME: LowPassNeosFilter::IsFilterResultValid
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
        /// FUNCTION NAME: LowPassNeosFilter::IsFloatValid
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
        LowPassNeosFilter();

        LowPassNeosFilter(LowPassNeosFilter &);

        LowPassNeosFilter & operator=(LowPassNeosFilter const&); // assign op. hidden
    };

};
#endif