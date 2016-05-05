////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file LowPassFilterFixPt.hpp
///
/// The base class for the low pass filters that use fixed point arithmetic and derived from the template base class.
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
        /// @param  ulAtoDResolutionBits   Bit resolution of ADC channels
        /// @param  ulNumberOfPoles        Number of poles the filter implements
        ///
        /// @return  None
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        LowPassFilterFixedPt(uint32_t ulCornerFreqHZ,
                             uint32_t ulSamplingPeriodUS,
                             uint32_t ulLagCoeffecient,
                             uint32_t ulAtoDResolutionBits,
                             uint32_t ulNumberOfPoles = DEFAULT_NUMBER_OF_POLES)
            : m_ulAtoDResolutionBits(ulAtoDResolutionBits),
              m_ulNumberOfFrcntlBits((sizeof(int) * CHAR_BIT) - ulAtoDResolutionBits - 1),
			  m_ulIntNumBitsInInt(NUMBER_OF_BITS_IN_SIGNED_LONG_VALS),
			  m_ulRoundOffValue(1 << ((sizeof(int) * CHAR_BIT) - ulAtoDResolutionBits - 2)),
              m_ulScaledIntegerLSBBitPos((sizeof(int) * CHAR_BIT) - ulAtoDResolutionBits - 1),
              m_ulIntMSBSet(MOST_SIGNIFICANT_BIT_OF_UINT_VALS),
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
        /// Virtual method for the applying the low pass filter. Called synchrously at a periodic rate to compute a
		///  filtered output.  The output is calculated utilizing the following difference equation:
		///  y(n) = y(n-1) + Lag Coefficient * ( x(n) - y(n-1) )
		///  Where:
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
        /// @param  slAdcValueRead     Raw ADC data read by the ADC driver
        /// @param  ulCornerFreqHZ     Corner Frequency for the filter in herz
        /// @param  rslFilterOutput    Output from filter difference equation
        ///
        /// @return  true when the filter was applied, 
		///          false when the difference equation can't be applied.  Occurs when filter yields an invalid value,
		///          the filter can produce an invalid value when addition of terms in the difference equation results
		///          in an overflow. A return of false is also returned when the filter isnt' ready to start.  The filter
		///          isn't ready to start when the filter has not been configured with a corner frequency in herz that is in
		///          the acceptable range.  The acceptable range of corner frequencies is determined at object instantiation
		///          time.  Also the filter isn't ready to start when it has not been configured with a sample period in 
		///          microseconds that is in the acceptable range.  The accepatble range of sample periods is determined at
		///          object instantiation time.  False is also returned when the filter can't be reconfigured. Calles to 
		///          ApplyFilter() with a corner frequency that is different than the current corner frequency used will
		///          result in an attempt to configure the filter with the saved sample period in microseconds and the 
		///          the input corner frequency in herz.  If the call to configure fails, reconfigure fails. 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ApplyFilter(int32_t slAtoDValueRead, uint32_t ulCornerFreqToFilterHZ, int32_t & rslFilterOutput);

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
        ///
        /// @return  true when the filter was configured, false when corner frequency amd sampling period are out of bounds
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool ConfigureFilter(uint32_t ulCornerFreqHZ, uint32_t ulSamplePeriodUS);

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
        /// FUNCTION NAME: LowPassFilterFixedPt::RetrieveIntegerPartOfScaleddNmbrWithFixedBinaryPt
        ///
        /// Retrieve the integer part of a scaled number with a fixed binary point
		///
        /// @par Full Description
        /// Retrieve the integer part of a scaled number which is the number that is left of the binary point
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  slScaledNumber    Scaled integer value
        /// @param  ulBinaryPtBitPos  Bit position of the binary point
        ///
        /// @return  Integer part to the left of the binary shifted to bit 0.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline int32_t  RetrieveIntegerPartOfScaleddNmbrWithFixedBinaryPt(int32_t  slScaledNumber, uint32_t ulBinaryPtBitPos) 
        { return (slScaledNumber >> ulBinaryPtBitPos); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::RetrieveFractionalPartOfScaleddNmbrWithFixedBinaryPt
        ///
        /// Retrieve the fractional part of a scaled number which is the number that is right of the binary point
        ///
        /// @par Full Description
        /// Retrieve the fractional part of a scaled number which is the number that is right of the binary point
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  slScaledNumber    Scaled integer value
        /// @param  ulScalefactor     Scale factor (number of fractional bits)
        ///
        /// @return  fractional part to the right of the binary point
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline int32_t  RetrieveFractionalPartOfScaleddNmbrWithFixedBinaryPt(int32_t  slScaledNumber, uint32_t ulScaleFactor) 
        { return (((1 << ulScaleFactor) - 1) & slScaledNumber); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ComputeProductOfFracttionalParts
        ///
        /// Compute the product of the fractional parts of a scaled number 
        ///
        /// @par Full Description
        /// Take product of fractional parts that are to the right of the binary point in a scaled fixed point number.
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  ulFracPart1       Fractional part of the multiplicand
        /// @param  ulFractPart2      Fractional part of the multiplier
        /// @param  ulBinaryPtBitPos  Bit position of the binary point
        ///
        /// @return  product of fractional parts to the right of the binary point
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline int32_t ComputeProductOfFracParts(uint32_t ulFracPrt1, uint32_t ulFracPrt2, uint32_t ulBinaryPtBitPos)
        { return (static_cast<int32_t>(((!ulFracPrt1 || !ulFracPrt2) ? 0 : (((ulFracPrt1 * ulFracPrt2) + (1 << ulBinaryPtBitPos)) >> ulBinaryPtBitPos)))); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ComputeProductOfIntPartAndFracPart
        ///
        /// Compute product of fractional parts and integer parts 
        ///
        /// @par Full Description
        /// Compute product of fractional parts and integer parts of different scaled fixed point numbers
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  slIntPart         Integer part 
        /// @param  ulFractPart       Fractional part 
        ///
        /// @return  product of integer and fractional parts
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline int32_t ComputeProductOfIntegerPartAndFractionalPart(int32_t slIntPart, uint32_t ulFracPart)
        { return (((!slIntPart || !ulFracPart) ? 0 : static_cast<int32_t>(slIntPart * ulFracPart))); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ComputeProductOfIntegerParts
        ///
        /// Compute the product of the integer parts of a scaled number 
        ///
        /// @par Full Description
        /// Compute product of integer parts that are to the left of the binary point in a scaled fixed point number.
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param  slIntPart1      Integer part 1
        /// @param  slIntPart2      Integer part 2 
        ///
        /// @return  product of integer and fractional parts
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline int32_t ComputeProductOfIntegerParts(int32_t slIntPart1, int32_t slIntPart2, uint32_t ulBinaryPtBitPos)
        { return (((!slIntPart1 || !slIntPart2) ? 0 : ((slIntPart1 * slIntPart2) << ulBinaryPtBitPos))); }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::ScaledMultiply
        ///
        /// Multiply scaled ints (ints with a binary point).
        ///
        /// @par Full Description
        /// Product of two scaled integers 
        ///
        /// @pre    object created.
        /// @post   Product of two scaled numbers with a fixed binary point computed.
        ///
        /// @param slMultiplcand scaled int with binary point at scaleFactor - 1
        /// @param slMultiplier  scaled int with binary point at scaleFactor - 1
        /// @param ulScaleFactor bit positon of LSB of int part, one to the left of the binary point
        ///
        /// @return  Product of multiplier * multiplicand
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        int32_t ScaledMultiply(int32_t slMultiplicand, int32_t slMultiplier, uint32_t ulScaleFactor);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// FUNCTION NAME: LowPassFilterFixedPt::GetNumberOfADCResolutionBits
        ///
        /// Get the resolution in bits of the ADC inputs
        ///
        /// @par Full Description
        /// Get the resolution in bits of the ADC inputs
        ///
        /// @pre    object created.
        /// @post   none.
        ///
        /// @param none
        ///
        /// @return  Number of ADC resolution bits
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        inline uint32_t GetNumberOfADCResolutionBits() { return m_ulAtoDResolutionBits; }

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
        /// @param  slScaledAtoD           Raw ADC data scaled to a binary point
        /// @param  slLagCoefficient       Coefficient of the lag term.
        /// @param  rslFilteredValue       Value of raw ADC data filtered
        ///
        /// @return  true when calculation occurred without error, false when the difference equation yields an invalid value
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        virtual bool CalcDiffEquation(int32_t   slScaledAtoD,
                                      int32_t   slLagCoefficient,
                                      int32_t & rslFilteredValue);
        // Maximum number of poles that filter can process
        static const uint32_t  MAX_NUMBER_OF_POLES = 4;
        
        // Approx of Pi * .000001 with 32 bits of fractional precision
        static const uint32_t  PI_OMEGA          = 13493;

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

        // Number of ADC resolution bits
        uint32_t m_ulAtoDResolutionBits;

        // Number of fractionas bits in the scaled integers.  
        // Places to the right of the binary point.
        uint32_t m_ulNumberOfFrcntlBits;

        // For rounding to least significant bit of scaled data
        uint32_t m_ulRoundOffValue;

        // Bit position of LSB of whole number part of scaled number
        uint32_t m_ulScaledIntegerLSBBitPos;

        // Most significant bit for ints is a one, all others are zero
        uint32_t m_ulIntMSBSet;

		// Number of bits in an integer 
		uint32_t m_ulIntNumBitsInInt;

		// inhibit default constructor, copy constructor, and assignment
        LowPassFilterFixedPt();

        LowPassFilterFixedPt(LowPassFilterFixedPt &);

        LowPassFilterFixedPt & operator=(LowPassFilterFixedPt const&); // assign op. hidden
    };
};
#endif

