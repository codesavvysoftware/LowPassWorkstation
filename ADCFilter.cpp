////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file ADCFilter.cpp
///
/// Implementation of the ADCFilter class
///
/// @see ADCFilter.hpp for a detailed description of this class.
///
/// @if REVISION_HISTORY_INCLUDED
/// @par Edit History
/// - thaley1   22-Apr-2016 Original Implementation
/// @endif
///
/// @ingroup ???
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
    bool ADCFilter::ApplyFilter(int32_t slAtoDValueRead, uint32_t ulCornerFreqToFilter,  int32_t & rslFilterOutput)
    {
        int32_t slScaledAtoD = slAtoDValueRead << 15;

        rslFilterOutput = slScaledAtoD;
            
        bool bFilteringIsNotEnabled = !IsFilteringEnabled();

        if (bFilteringIsNotEnabled)
        {
            return false;
        }

        bool bReconfigureFailure = !ReconfigureWithNewCornerFrequencey(ulCornerFreqToFilter);

        if (bReconfigureFailure)
        {
            return false;
        }

        bool bLowPassFilteringStarting = HasFilterRestarted(rslFilterOutput);

        if (bLowPassFilteringStarting )
        {
           return true;
        }

        int32_t slCurrentFilterOutput = slScaledAtoD;

        bool bFilterApplied = true;

        for (int32_t sl = 0;
                (sl < sizeof(m_slPole) / sizeof(uint32_t))
             && (sl < m_ulNumberOfPoles);
             sl++)
        {
            //  y(n) = y(n-1) + ((AtoDRead << (AtoDResolution - 1)) - y(n-1)) >>  m_FrequencyShiftFactor
            //
            // ((AtoDRead << (AtoDResolution - 1)) - y(n-1)
            int32_t slLagTerm = iCurrentFilterOutput - m_slPole[sl];

            bool bInvalidFilterOutput = !IsFilterOutputValid(iLagTerm, iCurrentFilterOutput, m_slPole[sl]);

            if (bInvalidFilterOutput)
            {
                slCurrentFilterOutput = slScaledAtoD;

                bFilterApplied = false;

                break;
            }
                    
            // ((AtoDRead << (AtoDResolution - 1)) - y(n-1)) >>  m_FrequencyShiftFactor
            if (m_ulFrequencyShiftFactor < 0)
            {
                slLagTerm <<= (-m_ulFrequencyShiftFactor);
            }
            else
            {
                slLagTerm >>= m_FrequencyShiftFactor;
            }

            //  y(n) = y(n-1) + ((AtoDRead << (AtoDResolution - 1)) - y(n-1)) >>  m_FrequencyShiftFactor
            slCurrentFilterOutput = m_slPole[i] + slLagTerm;

            bInvalidFilterOutput = IsThereOverflowFromAddSbtrct(m_slPole[i], slLagTerm, slCurrentFilterOutput);

            if (bInvalidFilterOutput)
            {
                slCurrentFilterOutput = slScaledAtoD;

                bFilterApplied = false;

                break;
            }

            // For multiple poles
            m_slPole[i] = slCurrentFilterOutput;
        }

        rslFilterOutput = slCurrentFilterOutput;

        return bFilterApplied;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FUNCTION NAME: ADCFilter::ConfigureFilter
    ///
    /// Configure filter difference equation coefficients
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool ADCFilter::ConfigureFilter(uint32_t ulCornerFreq, uint32_t ulSamplingPeriod)
    {
        bool    bFilterConfigured = true;

        int32_t slNumberOfIntBits = GetNumberOfBitsInInt();

        int32_t slBitsToRightOfIntMSBForNoShift = GetNumberOfBitsInInt();
            
        slBitsToRightOfIntMSBForNoShift = slNumberOfIntBits - NO_SHIFT_BIT_POS;

        int32_t slBitsToRightOfIntMSBForAtoDRsltn = slNumberOfIntBits - GetNumberOfADCResolutionBits();

        int slSmallestShiftFactor = slBitsToRightOfIntMSBForAtoDRsltn - slBitsToRightOfIntMSBForNoShift;

        switch (uiCornerFreq)
        {
        case FREQ_100_HZ:
                
            m_slFrequencyShiftFactor = slSmallestShiftFactor;

            break;

        case FREQ_50_HZ:
        
            m_slFrequencyShiftFactor = slSmallestShiftFactor + 1;

            break;

        case FREQ_25_HZ:
        
            m_slFrequencyShiftFactor = slSmallestShiftFactor + 2;

            break;

        case FREQ_10_HZ:
     
            m_slFrequencyShiftFactor = slSmallestShiftFactor + 3;

            break;

        case FREQ_5_HZ:
        
            m_slFrequencyShiftFactor = slSmallestShiftFactor + 4;

            break;

        case FREQ_1_HZ:
          
            m_slFrequencyShiftFactor = slSmallestShiftFactor + 6;

            break;

         default:
         
             bFilterConfigured = false;

            break;
        }

        if (bFilterConfigured)
        {
            SetCornerFreq(ulCornerFreq);
        }

        return bFilterConfigured;
    }
};