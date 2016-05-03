////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassNeosFilter.hpp
///
/// The base class for the low pass filters that use floating point arithmetic and derived from the template base class.
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
    static const float TWO_PI_SECONDS_PER_MICROSECOND = (2.0f * 3.1415927f * .000001f);

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
		/// @param  uiCornerFreq         Initial corner frequency 
        /// @param  uiSamplingPeriod     Sampling period for the filter
        /// @param  fLagCoefficient      InitialLagCoefficient
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
        LowPassNeosFilter( uint32_t uiCornerFreq,
                           uint32_t uiSamplingPeriod,
                           float    fLagCoefficient)
            : m_fPrevFilteredValue(0.0),
              m_fRemainder(0.0),
              LowPass(uiCornerFreq, uiSamplingPeriod, fLagCoefficient)
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
		/// @param  fAtoDValueRead           Raw ADC data read by the ADC driver
        /// @param  uiCornerFreqToFilter     Corner Frequency for the filter.
        /// @param  rfFilterOutput           Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ApplyFilter(float        fAtoDValueRead, 
                         uint32_t     uiCornerFreqToFilter, 
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
		/// @param  uiCornerFreq         Corner Frequency for the filter
        /// @param  uiSamplingPeriod     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ConfigureFilter(uint32_t uiCornerFreq, uint32_t uiSamplingPeriod); 

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
        /// @return  true when calculation occurred without error, false otherwise
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
        virtual void InitFilterDataForRestart(float InitialFilterOutput);

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
        /// @return  true when the result is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFilterOutputValid(float fDiffEqTerm1, float fDiffEqTerm2, float fFilterOutput);

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
        /// @return  true when input float val is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFloatValid(float f);

        // inhibit default constructor, copy constructor, and assignment
        LowPassNeosFilter();

        LowPassNeosFilter(LowPassNeosFilter &);

        LowPassNeosFilter & operator=(LowPassNeosFilter const&); // assign op. hidden
    };

};
#endif