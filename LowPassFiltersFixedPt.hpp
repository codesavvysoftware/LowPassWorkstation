////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFilterFixPt.hpp
///
/// The base class for the low pass filters that use fixed point arithmetic and derived from the template base class.
///
/// LowPass template class is instantiated with type int32_t.  The pure virtual methods needed to implemented with the following signatures
/// 
///        virtual void ApplyFilter(int32_t adcValueRead, uint32_t ulCornerFreqHZ, int32_t & rFilterOutput, bool & rbFilterAppliedSuccessfully);
///        virtual void ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplingPeriodHZ, bool rbFilterConfigured);
///        virtual voidl CalcDiffEquation(int32_t   ADCValue,
///                                      int32_t    LagCoefficient,
///                                      int32_t &  rulFilteredValue,
///                                      bool &     rbCalculateSuccess);
///        virtual void InitFilterDataForRestart(int32_t InitialFilterOutput);
///        virtual bool IsFilterOutputValid(int32_t Term1,  int32_t Term2, int32_t Result);
///
///
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
#ifndef LOW_PASS_FIXED_PT_HPP
#define LOW_PASS_FIXED_PT_HPP

// SYSTEM INCLUDES

// C PROJECT INCLUDES


// C++ PROJECT INCLUDES
#include "LowPass.hpp"

namespace LowPassFilters
{

    class LowPassFilterFixedPt : public LowPass<int32_t>
    {
    public:
        //**************************************************************************************************************
        // Public methods
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::LowPassFixedPt
        ///
        /// Constructor
        ///
        /// @par Full Description
        /// Base class contructor for the low pass filter using fixed point arithmetic
        ///
        /// @pre    none
        /// @post   Object created for low filters that inherit it.
        /// 
        /// @param  ulCornerFreqHZ         Initial corner frequency in herz
        /// @param  ulSamplingPeriodUS     Sampling period for the filter in microseconds
        /// @param  ulLagCoefficient       InitialLagCoefficient
        /// @param  ulADCResolutionBits    Bit resolution of ADC channels
        /// @param  ulNumberOfPoles        Number of poles the filter implements
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LowPassFilterFixedPt(uint32_t ulCornerFreqHZ,
                             uint32_t ulSamplingPeriodUS,
                             uint32_t ulLagCoeffecient,
                             uint32_t ulADCResolutionBits,
                             uint32_t ulNumberOfPoles = DEFAULT_NUMBER_OF_POLES)
            : m_ulNumberOfFrcntlBits(NUMBER_OF_BITS_IN_SIGNED_LONG_VALS - ulADCResolutionBits - 1),
              m_ulConfigFilterValForMovingFracBitsToFixedPtNumber(NUMBER_OF_BITS_IN_SIGNED_LONG_VALS + ulADCResolutionBits + 1),
              m_ulADCResolutionBits(ulADCResolutionBits),
              m_ulNumberOfPoles(ulNumberOfPoles),
              LowPass<int32_t>(ulCornerFreqHZ, ulSamplingPeriodUS, ulLagCoeffecient)
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::~LowPassFixedPt
        ///
        /// Destructor
        ///
        /// @par Full Description
        /// Base class destructor for the low pass filter using fixed point arithmetic
        ///
        /// @pre    object created for low filters that inherit it.
        /// @post   object destroyed.
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual ~LowPassFilterFixedPt()
        {
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ApplyFilter
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
        /// @pre    object created.
        /// @post   filter applied to input A to D value when filter is enabled.
        ///
        /// @param  slADCValueRead                Raw ADC data read by the ADC driver
        /// @param  ulCornerFreqHZ                Corner Frequency for the filter in herz
        /// @param  rslFilterOutput               Output from filter difference equation
        /// @param  rbFilterAppliedSuccessfully   true when the filter was applied, 
        ///                                       false when the difference equation can't be applied.  Occurs when filter yields an invalid value,
        ///                                       the filter can produce an invalid value when addition of terms in the difference equation results
        ///                                       in an overflow. A return of false is also returned when the filter isn't ready to start.  The filter
        ///                                       isn't ready to start when the filter has not been configured with a corner frequency in herz that is in
        ///                                       the acceptable range.  The acceptable range of corner frequencies is determined at object instantiation
        ///                                       time.  Also the filter isn't ready to start when it has not been configured with a sample period in 
        ///                                       microseconds that is in the acceptable range.  The accepatble range of sample periods is determined at
        ///                                       object instantiation time.  False is also returned when the filter can't be reconfigured. Calls to 
        ///                                       ApplyFilter() with a corner frequency that is different than the current corner frequency used will
        ///                                       result in an attempt to configure the filter with the saved sample period in microseconds and the 
        ///                                       the input corner frequency in herz.  If the call to configure fails, reconfigure fails. 
        ///
        /// @return  true when the filter was applied, 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void ApplyFilter(int32_t slADCValueRead, uint32_t ulCornerFreqToFilterHZ, int32_t & rslFilterOutput, bool & bFilterAppliedSuccessfully);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ConfigureFilter
        ///
        /// Configure filter difference equation coefficients
        ///
        /// @par Full Description
        /// Virtual method for the configuring the low pass filter
        ///
        /// @pre    object created.
        /// @post   filter configure to input corner frequency and sample period.
        ///
        /// @param  uiCornerFreqHZ         Corner Frequency for the filter herz
        /// @param  uiSamplingPeriodUS     Sampling period for the filter in microseconds
        /// @param  rbFilterConfigured     true when the filter was configured, false when corner frequency amd sampling 
        ///                                period are out of bounds.
        /// 
        /// @return return none
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplePeriodUS, bool & rbFilterConfigured);

    protected:
        //**************************************************************************************************************
        // Protected methods and attributes
        //**************************************************************************************************************
    
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::InitFilterDataForRestart
        ///
        /// Initialize filter data to put the filter in it's initial state
        ///
        /// @par Full Description
        /// virtual method for initializing filter data.
        ///
        /// @pre    object created.
        /// @post   filter parameters for restarting the filter initialized.
        ///
        /// @param  slIntialFilterOutput  Initial value of the filter when restarting
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void InitFilterDataForRestart(int32_t slInitialFilterOutput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::IsFilterResultValid
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Method for determining the validity of the low filter output generated by difference equation
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  slDiffEqTerm1     First term of difference equation add
        /// @param  slTerm2           Second term of difference equation add
        /// @param  slResult          Result of difference equation add
        ///
        /// @return  true when the result is valid, false when applying the filter causes an overflow.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsFilterOutputValid(int32_t slDiffEqTerm1, int32_t slDiffEqTerm2, int32_t slFilterOuput);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::IsThereOverflowFromAddSbtrct
        ///
        /// Determine validity of low pass filter output
        ///
        /// @par Full Description
        /// Method for determining if overflow occurred during a binary add/subtract
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  ulTerm1           First term of binary aritmetic operation of filter being checked
        /// @param  ulTerm2           Second term of binary aritmetic operation of filter being checked.
        /// @param  ulResult          Result of binary arithmetic operation of filter being checked
        ///
        /// @return  true when overflow occurred in the addition/subtraction, false when overflow occurs
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsThereOverflowFromAddSbtrct(uint32_t ulTerm1, uint32_t ulTerm2, uint32_t ulResult);


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::IsADCResolutionInRange
        ///
        /// Determine of ADC resolution bits within allowable range.
        ///
        /// @par Full Description
        /// Method for determining if the ADC resolution specified is within allowable range 
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param none
        ///
        /// @return true when ADC resolution bit within allowable range, false when not
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool IsADCResolutionInRange();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfFractionalBits
        ///
        /// Get the number of fractional bits for scaled fixed point integers 
        ///
        /// @par Full Description
        /// Get the number of fractional bits for scaled fixed point integers which is derived from the ADC resolution
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param none
        ///
        /// @return  Number fractional bits
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline uint32_t GetNumberOfFractionalBits() { return (m_ulNumberOfFrcntlBits); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ScaledMultiply
        ///
        /// Multiply two fixed point ints with a fractional part at a binary point.
        ///
        /// @par Full Description
        /// Product of two ixed point ints with a fractional part at a binary point 
        ///
        /// @pre    object created.
        /// @post   Product of fixed point ints with a fractional part at a binary point.
        ///
        /// @param slMultiplcand             fixed point int with a fractional part at a binary point
        /// @param slMultiplier              fixed point int with a fractional part at a binary point
        /// @param ulNumberOfFractionalBits  bit positon of LSB of int part, one to the left of the binary point
        ///
        /// @return  Product of multiplier * multiplicand
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        int32_t ScaledMultiply(int32_t slMultiplicand, int32_t slMultiplier, uint32_t ulNumberOfFractionalBits);

 
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
        /// @param  slScaledADCValue       Raw ADC data scaled to a binary point
        /// @param  slLagCoefficient       Coefficient of the lag term.
        /// @param  rslFilteredValue       Value of raw ADC data filtered
        /// @param  rbCaluculateSuccess    true when calculation occurred without error, false when the difference equation yields an invalid value
        ///
        /// @return none
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual void CalcDiffEquation(int32_t   slScaledADCValue,
                                      int32_t   slLagCoefficient,
                                      int32_t & rslFilteredValue,
                                      bool &    rbCalculateSuccess);

        // Maximum number of poles that filter can process
        static const uint32_t  MAX_NUMBER_OF_POLES = 4;
        
        // Approx of Pi * .000001 with 32 bits of fractional precision
        static const uint32_t  LAG_CONSTANT = 13493;

        // For rounding off 64 bit fractional value;
        static const uint64_t  ROUND_OFF_FRAC_64 = 0x800000000000;

        // Number of poles in the filter
        uint32_t m_ulNumberOfPoles;

        // Poles used in applying the filter.
        int32_t  m_slPole[MAX_NUMBER_OF_POLES];

    private:
        //**************************************************************************************************************
        // Private methods and attributes
        //**************************************************************************************************************
    
        // Default number of poles to use in filtering
        const static uint32_t DEFAULT_NUMBER_OF_POLES = 1;

        // One point zero scaled with a binary point at 2**31
        const static uint64_t ONE_PT_ZERO_SCALED_BINARY_PT = 0x100000000;
        
        // Bit position of most significant bit in unsigned int vals.
        const static uint32_t MOST_SIGNIFICANT_BIT_OF_UINT_VALS = static_cast<uint32_t>(INT_MAX) + 1;

        // Number of bits in an int value
        const static uint32_t NUMBER_OF_BITS_IN_SIGNED_LONG_VALS = sizeof(int) * CHAR_BIT;

        // Maximum number of bits of A/D resolution
        const static uint32_t MAX_ADC_RESOLUTION_BITS = 26;

        // Minimum number of bits of A/D resolution
	const static uint32_t MIN_ADC_RESOLUTION_BITS = 8;

        // Number of fractionas bits in the scaled integers.  
        // Places to the right of the binary point.
        uint32_t m_ulNumberOfFrcntlBits;

        // Used for shifting the lag coefficient calculated as in ConfigFilter to the correct bit position.
        uint32_t m_ulConfigFilterValForMovingFracBitsToFixedPtNumber;

        // Number of ADC Resolution Bits
        uint32_t m_ulADCResolutionBits;

        // inhibit default constructor, copy constructor, and assignment
        LowPassFilterFixedPt();

        LowPassFilterFixedPt(LowPassFilterFixedPt &);

        LowPassFilterFixedPt & operator=(LowPassFilterFixedPt const&); // assign op. hidden
    };
};
#endif

