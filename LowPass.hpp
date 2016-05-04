////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPass.hpp
///
/// This class characterizes the aborted state for a firmware update.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley 03-May-2016 Original implementation
/// @endif
///
/// @ingroup ???
///
/// @par Copyright (c) 2016 Rockwell Automation Technolgies, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LOWPASS_HPP
#define	LOWPASS_HPP

// SYSTEM INCLUDES
// (none)

// C PROJECT INCLUDES
// (none)
#include <math.h>
#include <stdint.h>

// C++ PROJECT INCLUDES
#include <climits>

// FORWARD REFERENCES
// (none)


namespace LowPassFilters
{
    template<typename NumericFormat> class LowPass
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::LowPass
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Base class contructor for the Low Pass Single Order Lag Filters
        ///
        /// @pre    none
        /// @post   Base class object created for low filters that inherit it.
        /// 
        /// @param  ulCornerFreqHZ       Initial corner frequency herz
        /// @param  ulSamplingPeriodUS   Sampling period for the filter microseconds
        /// @param  LagCoefficient       InitialLagCoefficient
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LowPass(uint32_t       ulCornerFreqHZ,
                uint32_t       ulSamplingPeriodUS,
                NumericFormat  LagCoefficient,
                uint32_t       ulCornerFreqUpperBoundHZ = DEFAULT_CORNER_FREQ_UPPER_BOUND_HZ,
                uint32_t       ulCornerFreqLowerBoundHZ = DEFAULT_CORNER_FREQ_LOWER_BOUND_HZ,
                uint32_t       ulSamplingPeriodUpperBoundUS = DEFAULT_SAMPLE_PERIOD_UPPER_BOUND_US,
                uint32_t       ulSamplingPeriodLowerBoundUS = DEFAULT_SAMPLE_PERIOD_LOWER_BOUND_US )
            : m_ulCornerFreqHZ(ulCornerFreqHZ),
              m_ulSamplePeriodUS(ulSamplingPeriodUS),
              m_LagCoefficient(LagCoefficient),
              m_bFilteringEnabled(false),
              m_bValidConfigurationActive(false),
              m_bFirstSample(true),
              m_bFilterConfigurationValid(false),
              m_ulCornerFreqUpperBnd(ulCornerFreqUpperBoundHZ),
              m_ulCornerFreqLowerBnd(ulCornerFreqLowerBoundHZ),
              m_ulSamplingPeriodUpperBnd(ulSamplingPeriodUpperBoundUS),
              m_ulSamplingPeriodLowerBnd(ulCornerFreqLowerBoundHZ)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::~LowPass
        ///
        /// Destrcutor
        ///
        /// @pre    Base class object created for low filters that inherit it.
        /// @post   Base class object destroyed.
        /// 
        /// @par Full Description
        /// Base class destructor for the Low Pass Single Order Lag Filters
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~LowPass()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::ApplyFilter
        ///
        /// Apply the low pass filter difference equation to unfiltered ADC input data
        ///
        /// @par Full Description
        /// Pure virtual method for the applying the low pass filter
        ///
        /// @pre    none.
        /// @post   None but all filters that inherit the base class must implement a method with this signature.
        /// 
        /// @param  adcValueRead     Raw ADC data read by the ADC driver
        /// @param  ulCornerFreqHZ   Corner Frequency for the filter in Herz.
        /// @param  rFilterOutput    Output from filter difference equation
        ///
        /// @return  true when the filter was applied, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ApplyFilter(NumericFormat adcValueRead, uint32_t ulCornerFreqHZ, NumericFormat & rFilterOutput) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Pure virtual method for the configuring the low pass filter
        ///
        /// @pre    none.
        /// @post   None but all filters that inherit the base class must implement a method with this signature.
        /// 
        /// @param  ulCornerFreqHZ         Corner Frequency for the filter in herz
        /// @param  ulSamplingPeriodUS     Sampling period for the filter in microseconds
        ///
        /// @return  true when the filter was configured, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodHZ) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::EnableFiltering()
        ///
        /// Enable low pass filtering
        ///
        /// @par Full Description
        /// Enable low pass filtering of raw ADC data.
        ///
        /// @pre    none.
        /// @post   Filtering enabled.
        /// 
        /// @return None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void EnableFiltering() { m_bFilteringEnabled = true; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::DisableFiltering()
        ///
        /// Disable low pass filtering
        ///
        /// @par Full Description
        /// Disable low pass filtering of raw ADC data.
        ///
        /// @pre    none.
        /// @post   Filtering disabled.
        /// 
        /// @return None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void DisableFiltering() { m_bFilteringEnabled = false; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsFilteringEnabled()
        ///
        /// Query filtering enabled status
        ///
        /// @par Full Description
        /// Query filtering enabled status to determine if filtering is enabled or disabled.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return true when filtering is enabled, false when disabled
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline bool IsFilteringReadyToStart() { return ( m_bFilteringEnabled && m_bValidConfigurationActive); }

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
        /// @param  AtoDValue           Raw ADC data scaled to a binary point
        /// @param  LagCoefficient      Coefficient of the lag term.
        /// @param  FilteredValue       Value of raw ADC data filtered
        ///
        /// @return  true when calculation occurred without error, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool CalcDiffEquation(NumericFormat   AtoDValue,
                                      NumericFormat   LagCoefficient,
                                      NumericFormat & FilteredValue) = 0; 

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::InitFilterDataForRestart
        ///
        /// Initialize filter data to put the filter in it's initial state
        ///
        /// @par Full Description
        /// Pure virtual method for initializing filter data.
        ///
        /// @pre    none.
        /// @post   None but all filters that inherit the base class must implement a method with this signature.
        /// 
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void InitFilterDataForRestart(NumericFormat InitialFilterOutput) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsFilterResultValid
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Pure virtual method for determining the validity of the low filter output generated by difference equation
        ///
        /// @pre    none.
        /// @post   None but all filters that inherit the base class must implement a method with this signature.
        /// 
        /// @param  Term1           First term of binary aritmetic operation of filter being checked
        /// @param  Term2           Second term of binary aritmetic operation of filter being checked.
        /// @param  Result          Result of binary arithmetic operation of filter being checked
        ///
        /// @return  true when the result is valid, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool IsFilterOutputValid(NumericFormat Term1, NumericFormat Term2, NumericFormat Result) = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::HasFilterRestarted
        ///
        /// Determine if the filter has started/restarted
        ///
        /// @par Full Description
        /// Determining if the filter has started/restarted. When starting restarting put filter in not restarting state
        /// and initialize the filter data to the starting state.
        ///
        /// @pre    Filter object instantiated.
        /// @post   Filter data for the filter object initialized.
        /// 
        /// @param  rInitialFilterOutput    For return fromm appying filter difference equation and initializing data
        ///
        /// @return  true when the filter is restarting, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool HasFilterRestarted(NumericFormat rInitialFilterOutput)
        {
            bool bFilterRestarted = false;

            if (m_bFirstSample)
            {
                InitFilterDataForRestart(rInitialFilterOutput);

                m_bFirstSample = false;

                bFilterRestarted = true;
            }

            return bFilterRestarted;
        }

       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::ReconfigureWithNewCornerFrequencey
        ///
        /// Reconfigure low pass filter
        ///
        /// @par Full Description
        /// Reconfigure low pass filter synchronously, when applying filter.
        ///
        /// @pre    Filter instantiated.
        /// @post   The filter is reconfigured with a new lag coefficient with valid sample period and corner frequency
        /// 
        /// @param  ulCornerFreqForFilterHZ     Corner frequency to reconfigure filter with in herz
        ///
        /// @return  true when filter reconfigured successfully, false otherwise
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool ReconfigureWithNewCornerFrequencey(uint32_t ulCornerFreqToFilterHZ)
        {
            uint32_t ulCornerFreqHZ = GetCornerFreqHZ();

            bool bConfigureSuccess = true;

            if (ulCornerFreqToFilterHZ != ulCornerFreqHZ)
            {
                uint32_t ulSamplePeriodUS = GetSamplePeriodUS();

                bConfigureSuccess = ConfigureFilter(ulCornerFreqToFilterHZ, ulSamplePeriodUS);
            }

            return bConfigureSuccess;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsCornerFreqWithBounds()
        ///
        /// Query if conrner frequency is within bounds
        ///
        /// @par Full Description
        /// Query if conrner frequency is within the upper and lower bounds of its valid values
        /// frequency.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return true when corner frequency is within valid bounds, false when disabled
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline bool IsCornerFreqWithBounds(uint32_t ulCornerFreqHZ)
        { return ((ulCornerFreqHZ <= m_ulCornerFreqUpperBnd) && (ulCornerFreqHZ >= m_ulCornerFreqLowerBnd)); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsSamplingPeriodWithBounds()
        ///
        /// Query if sampling period is within bounds
        ///
        /// @par Full Description
        /// Query if sampling period is within the upper and lower bounds of its valid values
        /// frequency.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return true when sampling period is within valid bounds, false when disabled
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline bool IsSamplingPeriodWithBounds(uint32_t ulSamplingPeriodUS)
        { return ((ulSamplingPeriodUS <= m_ulSamplingPeriodUpperBnd) && (ulSamplingPeriodUS >= m_ulSamplingPeriodLowerBnd)); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::IsFilterConfigured()
        ///
        /// Query filtering enabled status
        ///
        /// @par Full Description
        /// Query filtering enabled status to determine if filtering has valid configuration of sample period and corner
        /// frequency.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return true when filtering is enabled, false when disabled
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline bool IsFilterConfigured() 
        { return m_bFilterConfigurationValid; }

        inline void SetFilteringConfigured()
        {	m_bFilterConfigurationValid = true;	}
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void ResetFilteringConfigured()
        { m_bFilterConfigurationValid = false; }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::GetCornerFreq
        ///
        /// Get corner frequency
        ///
        /// @par Full Description
        /// Get corner frequency the filter is configured for.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return  The corner frequency in herz the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline uint32_t GetCornerFreqHZ() { return m_ulCornerFreqHZ; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::GetSamplePeriod
        ///
        /// Get sample period
        ///
        /// @par Full Description
        /// Get sample period the filter is configured for.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return  The sample period in microseconds the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline uint32_t GetSamplePeriodUS() 
        { return m_ulSamplePeriodUS; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::GetLagCoefficient
        ///
        /// Get lag coefficient
        ///
        /// @par Full Description
        /// Get lag coefficient the filter is configured for.
        ///
        /// @pre    none.
        /// @post   none.
        /// 
        /// @return  The lag coefficient the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline NumericFormat GetLagCoefficient() 
        { return m_LagCoefficient; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::RestartFiltering
        ///
        /// Restart Low Pass Filtering
        ///
        /// @par Full Description
        /// Restart digitial filtering from first sample
        ///
        /// @pre    none.
        /// @post   m_bFirstSample is set indicating to ApplyFilter that filtering has restarted.
        /// 
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void RestartFiltering() 
        { m_bFirstSample = false; }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::SetCornerFreq
        ///
        /// Set Corner Frequency
        ///
        /// @par Full Description
        /// Set new corner frequency to input corner frequency.
        ///
        /// @pre    none.
        /// @post   New corner frequency saved.
        /// 
        /// @param  ulCornerFreq     Corner frequency in herz for the filter
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void SetCornerFreqHZ(uint32_t ulCornerFreqHZ) 
        { m_ulCornerFreqHZ = ulCornerFreqHZ; }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::SetLagCoefficient
        ///
        /// Set lag coefficient
        ///
        /// @par Full Description
        /// Set new lag coefficient to input lag coefficient.
        ///
        /// @pre    none.
        /// @post   New lag coefficient saved.
        /// 
        /// @param  LagCoefficient     Lag coefficient for the filter
        ///
        /// @return  The lag coefficient the filter is configured for
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void SetLagCoefficient(NumericFormat LagCoefficient) 
        { m_LagCoefficient = LagCoefficient; }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPass::SetSamplingPeriod
        ///
        /// Set Corner Frequency
        ///
        /// @par Full Description
        /// Set new sampling period to input sampling period.
        ///
        /// @pre    none.
        /// @post   New sampling period in microseconds saved.
        /// 
        /// @param  uiSamplingPeriod     Sampling period for the filter
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline void SetSamplingPeriodUS(uint32_t ulSamplingPeriodUS) 
        { m_ulSamplePeriodUS = ulSamplingPeriodUS; }

    private:
        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
        // Default max value for corner freq in herz
        static const uint32_t   DEFAULT_CORNER_FREQ_UPPER_BOUND_HZ = 500;
        
        // Default min value for corner freq in herz
        static const uint32_t   DEFAULT_CORNER_FREQ_LOWER_BOUND_HZ = 1;
        
        // Default max value for sample period in microseconds
        static const uint32_t   DEFAULT_SAMPLE_PERIOD_UPPER_BOUND_US = 1000;
        
        // Default min value for sample period in microseconds
        static const uint32_t   DEFAULT_SAMPLE_PERIOD_LOWER_BOUND_US = 10;

        // true when applying difference equation to filter is active
        bool           m_bFilteringEnabled;

        // true when filter is configured with valid corner freq and sample period
        bool           m_bValidConfigurationActive;

        // corner frequency for the filter
        uint32_t       m_ulCornerFreqHZ;

        // sampling period for the filter
        uint32_t       m_ulSamplePeriodUS;

        // lag coefficient for the filter
        NumericFormat  m_LagCoefficient;

        // true when the first sample for a restarted has been read
        bool           m_bFirstSample;

        // true when the filter is configured with a valid sample period and corner frequency
        bool           m_bFilterConfigurationValid;

        // Max value for corner freq that filter can be configured with
        uint32_t       m_ulCornerFreqUpperBnd;

        // Min value for corner freq that filter can be configured with
        uint32_t       m_ulCornerFreqLowerBnd;
            
        // Max value for sampling rate that filter can be configured with
        uint32_t       m_ulSamplingPeriodUpperBnd;

        // Min value for sampling rate that filter can be configured with
        uint32_t       m_ulSamplingPeriodLowerBnd;

        
        // inhibit default constructor, copy constructor, and assignment
        LowPass();

        LowPass(LowPass &);
        
        LowPass & operator=(LowPass const&); // assign op. hidden
    };

};
#endif


