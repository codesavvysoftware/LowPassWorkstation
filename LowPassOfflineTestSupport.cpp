#include <math.h>
#include <stdint.h>
#include "LowPassOfflineTestSupport.hpp"

namespace LowPassOfflineTesting
{
    double ConvertScaledNumToReal(int32_t ScaledNum, unsigned int ScaleFactor)
    {
        unsigned int Mask = 1 << (ScaleFactor-1);

        Mask -= 1;

        unsigned int Mask0 = Mask ^ 0xffffffff;

        unsigned int WholeNum = (ScaledNum & Mask0) >> (ScaleFactor-1);

        double WholeNumPart = WholeNum;

        unsigned int Frac = ScaledNum;

        Frac &= Mask;

        double FracPart = Frac;

        double denominator = pow(2.0, (double)(ScaleFactor-1));

        FracPart *= (1.0 / denominator);

        double RealVal = WholeNumPart + FracPart;

        return RealVal;
    }

    bool GetScaledVal(double Value, int32_t & ScaledNum, uint32_t ScaleFactor)
    {
        if (Value == 0.0)
        {
            ScaledNum = 0;

            return true;
        }
        double     accum       = Value;

        int32_t   IntegerPart = Value;

        double dWholeNumPart = IntegerPart;

        accum -= dWholeNumPart;

        uint32_t scaledval = 0;

        int i = 0;

        double PowerOf2;

        for (i = 0; i < ScaleFactor - 1; i++)
        {
            PowerOf2 = 1.0;

            PowerOf2 /= (2 << i);

            double interVal = accum - PowerOf2;

            scaledval <<= 1;

            if (interVal >= 0.0)
            {
                scaledval |= 1;

                accum = interVal;
            }

            //if (interVal == 0.0) break;

        }
        
        ScaledNum = (IntegerPart << ScaleFactor - 1) | scaledval;

        return true;

    }


    void InitFilterData(FILTER_TEST_RESULTS & filterResults, double const * &dInputVals, double dInitialVal, uint32_t Cutoff, uint32_t Period)
    {
        for (uint32_t ui = 0; ui < (sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL)); ui++)
        {
            filterResults.ftv[ui].dInput = dInputVals[ui];

            filterResults.ftv[ui].dOutput = -1.0;
        }

        filterResults.uiCutoff = Cutoff;

        filterResults.uiPeriod = Period;

        filterResults.dInitialVal = dInitialVal;

    } 

    void RunFilterForCheckingLowPass(FILTER_TEST_RESULTS & filterResults)
    {
        static const double PI = 3.1415927;

        double Cutoff = filterResults.uiCutoff;

        double Period = filterResults.uiPeriod;

        double Radians = 2.0 * PI * Cutoff * Period * 0.000001;
        
        double calcLagCoeff = Radians / (Radians + 2.0);

        double dPrevOutput = filterResults.dInitialVal;

        double dCurrentOutput = 0.0;

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) /  (sizeof(FILTER_TEST_VAL));

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
           filterResults.ftv[ui].dOutput = dPrevOutput + (calcLagCoeff * (filterResults.ftv[ui].dInput - dPrevOutput));

           dPrevOutput = filterResults.ftv[ui].dOutput;
        }
        
    }

    void RunAdcFilter(FILTER_TEST_RESULTS & filterResults)
    {
        static const double PI = 3.1415927;

        double Cutoff = filterResults.uiCutoff;

        double Period = filterResults.uiPeriod;

        double Radians = 2.0 * PI * Cutoff * Period * 0.000001;

        double calcLagCoeff = Radians / (Radians + 2.0);

        int32_t ScaledLagCoeff = 0;

        GetScaledVal(calcLagCoeff, ScaledLagCoeff);

        ADCFilter ADCTest(filterResults.uiCutoff,
            filterResults.uiPeriod,
            ScaledLagCoeff,
            16);

        ADCTest.EnableFiltering();

        int32_t CurrentAtoDReading = 0;

        GetScaledVal(filterResults.dInitialVal, CurrentAtoDReading);

        int32_t iOutput;

        ADCTest.ConfigureFilter(100, 50);

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            GetScaledVal(filterResults.ftv[ui].dInput, CurrentAtoDReading);

            ADCTest.ApplyFilter((CurrentAtoDReading >> 15), filterResults.uiCutoff, iOutput);

            filterResults.ftv[ui].dOutput = ConvertScaledNumToReal(iOutput);
        }

    }

    void RunNeosFilter(FILTER_TEST_RESULTS & filterResults)
    {
        static const double PI = 3.1415927;

        double Cutoff = filterResults.uiCutoff;

        double Period = filterResults.uiPeriod;

        double Radians = 2.0 * PI * Cutoff * Period * 0.000001;

        double calcLagCoeff = Radians / (Radians + 2.0);

        int32_t ScaledLagCoeff = 0;

        GetScaledVal(calcLagCoeff, ScaledLagCoeff);

        LowPassNeosFilter NeosTest(filterResults.uiCutoff,
                                   filterResults.uiPeriod,
                                   calcLagCoeff);

        NeosTest.EnableFiltering();

        float CurrentAtoDReading = 0.0f;

        float fOutput;

        NeosTest.ConfigureFilter(100, 50);

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            CurrentAtoDReading = filterResults.ftv[ui].dInput;

            NeosTest.ApplyFilter(CurrentAtoDReading, filterResults.uiCutoff, fOutput);

            filterResults.ftv[ui].dOutput = fOutput;
        }

    }
    void RunFixedPtFilter(FILTER_TEST_RESULTS & filterResults)
    {
        static const double PI = 3.1415927;

        double Cutoff = filterResults.uiCutoff;

        double Period = filterResults.uiPeriod;

        double Radians = 2.0 * PI * Cutoff * Period * 0.000001;

        double calcLagCoeff = Radians / (Radians + 2.0);

        int32_t ScaledLagCoeff = 0;

        GetScaledVal(calcLagCoeff, ScaledLagCoeff);

        LowPassFilterFixedPt FixedPtTest(filterResults.uiCutoff,
                                         filterResults.uiPeriod,
                                         calcLagCoeff,
                                         16);

        FixedPtTest.EnableFiltering();

        int32_t CurrentAtoDReading = 0;

        int32_t sOutput;

        FixedPtTest.ConfigureFilter(100, 50);

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            CurrentAtoDReading = filterResults.ftv[ui].dInput;

            FixedPtTest.ApplyFilter(CurrentAtoDReading, filterResults.uiCutoff, sOutput);

            filterResults.ftv[ui].dOutput = ConvertScaledNumToReal(sOutput);
        }

    }
    void RunOUVFilter(FILTER_TEST_RESULTS & filterResults)
    {
        static const double PI = 3.1415927;

        double Cutoff = filterResults.uiCutoff;

        double Period = filterResults.uiPeriod;

        double Radians = 2.0 * PI * Cutoff * Period * 0.000001;

        double calcLagCoeff = Radians / (Radians + 2.0);

        int32_t ScaledLagCoeff = 0;

        GetScaledVal(calcLagCoeff, ScaledLagCoeff);
        
        OUVFilter  OUVTest(filterResults.uiCutoff, filterResults.uiPeriod, ScaledLagCoeff, 16);

        OUVTest.EnableFiltering();

        int32_t CurrentAtoDReading = 0;

        GetScaledVal(filterResults.dInitialVal, CurrentAtoDReading);

        int32_t iOutput;

        //OUVTest.ApplyFilter((CurrentAtoDReading >> 15), filterResults.uiCutoff, iOutput);

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            GetScaledVal(filterResults.ftv[ui].dInput, CurrentAtoDReading);

            OUVTest.ApplyFilter((CurrentAtoDReading>>15), filterResults.uiCutoff, iOutput);

            filterResults.ftv[ui].dOutput = ConvertScaledNumToReal(iOutput);
        }     

    }

    void CompareFilterOutputs(FILTER_TEST_RESULTS & ftrCompareTo, FILTER_TEST_RESULTS & ftrTargetBeingTested, double tolerancePercentAllowed)
    {
        uint32_t uiNumberOfInputs = sizeof(ftrCompareTo.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            double diffComputedKnown = ftrCompareTo.ftv[ui].dOutput;

            diffComputedKnown -= ftrTargetBeingTested.ftv[ui].dOutput;

            if (diffComputedKnown < 0.0)
            {
                diffComputedKnown = -diffComputedKnown;
            }

            diffComputedKnown /= ftrCompareTo.ftv[ui].dOutput;

            diffComputedKnown *= 100.0;

            ftrTargetBeingTested.ftv[ui].dPercentError = diffComputedKnown;

            if (diffComputedKnown <= tolerancePercentAllowed)
            {
                ftrTargetBeingTested.ftv[ui].bInTolerance = true;
            }
            else
            {
                ftrTargetBeingTested.ftv[ui].bInTolerance = false;

            }
        }

    }
};