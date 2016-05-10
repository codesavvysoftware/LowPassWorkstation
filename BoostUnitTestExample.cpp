// BoostUnitTestExample.cpp : Defines the entry point for the console application.
//

#define BOOST_TEST_ALTERNATIVE_INIT_API
#include "stdafx.h"


#define BOOST_TEST_MODULE LowPassFilter_UI

#define  BOOST_TEST_NO_MAIN

#include <boost/test/included/unit_test.hpp>

#include "OUVFilter.hpp"
#include "LowPassFloat.hpp"
#include "LowPassFiltersFixedPt.hpp"
#include "ADCFilter.hpp"

//#include "LowPassFiltering.hpp"
//#define BOOST_NO_CXX11_SCOPED_ENUMS
//#include <boost/filesystem.hpp>
//#undef BOOST_NO_CXX11_SCOPED_ENUMS //#include <boost/filesystem.hpp>
//#include <fstream>

//#include <direct.h>
#define chdir _chdir

using namespace boost::unit_test;
using namespace LowPassFilters;

struct LogToFile
{
    LogToFile()
    {
        std::string logFileName(boost::unit_test::framework::master_test_suite().p_name);
        logFileName.append(".xml");
        logFile.open(logFileName.c_str());
        boost::unit_test::unit_test_log.set_stream(logFile);
    }
    ~LogToFile()
    {
        boost::unit_test::unit_test_log.test_finish();
        logFile.close();
        boost::unit_test::unit_test_log.set_stream(std::cout);
    }
    std::ofstream logFile;
};

BOOST_GLOBAL_FIXTURE(LogToFile);

int main(int  argc,
         char **  argv
    )
{
    char * argv0 = argv[0];

    return boost::unit_test::unit_test_main(&init_unit_test, argc, argv);

}
//using namespace LowPassFilteringSpace;

const char * progName = "LowPassFilter";

static const float PI = 3.1415927f;

static const float Cutoff = 100.0f;

static const float Period = .000001f * 50.0f;

static const float fRadiansTC = 2.0f * PI * Cutoff * Period;

static const float LagCoeff = fRadiansTC / (fRadiansTC + 2.0f);

static const unsigned int NUMBER_OF_FILTER_TEST_VALS = 10;

static const double SampleInputSteadState[NUMBER_OF_FILTER_TEST_VALS] = { 10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0 };

static const double SampleInputRamp[NUMBER_OF_FILTER_TEST_VALS] = { 1.0,2.0,4.0,8.0,16.0,32.0,64.0,128.0,256.0,512.0 };

static const double * pSteadyStateSample = SampleInputSteadState;

static const double * pRampSample = SampleInputRamp;

typedef struct
{
    double dInput;

    double dOutput;

    double dPercentError;

    double dTolerance;

    bool   bInTolerance;

} FILTER_TEST_VAL;

typedef struct
{
    FILTER_TEST_VAL   ftv[NUMBER_OF_FILTER_TEST_VALS];

    double            dInitialVal;

    uint32_t          uiCutoff;

    uint32_t          uiPeriod;
} FILTER_TEST_RESULTS;

static FILTER_TEST_RESULTS   knownGoodFilterResults;

static FILTER_TEST_RESULTS   OuvFilterResults;

static FILTER_TEST_RESULTS   ADCFilterResults;

static FILTER_TEST_RESULTS   FloatFilterResults;

static FILTER_TEST_RESULTS   FixedPtResults;

typedef struct
{
    unsigned int        uiCornerFreqHZ;
    unsigned int        uiSamplePeriodUS;
    double              dActualCalcErrDiffThreshold;
    const double *      pdInputData;
    unsigned int        uiNumberOfInputDataItems;
}
FILTER_TEST_PARAMETERS;


struct LowPassFilter_fixture
{
    FILTER_TEST_PARAMETERS default_params{ 100,50,3.0,pRampSample,NUMBER_OF_FILTER_TEST_VALS };

    FILTER_TEST_PARAMETERS current_params = default_params;
    
    void ConfigureBasicTestParameters(unsigned int uiTestCornerFreqHZ,            // CornerFrequency HZ
                                      unsigned int uiTestSamplePeriodUS,          // Sample Period US
                                      double       dDiffActualCalcErrThreshold,   // Differance between actual and caculated filter values passing thresholdunsigned int uiCornerFreqHZ = 100;
                                      double * &   dpInputDataToFilter,
                                      unsigned int uiNumberOfInputDataItems)          // Data to Apply the filter to.
    {
        current_params.uiCornerFreqHZ = uiTestCornerFreqHZ;
        current_params.uiSamplePeriodUS = uiTestSamplePeriodUS;
        current_params.dActualCalcErrDiffThreshold = dDiffActualCalcErrThreshold;
        current_params.pdInputData = dpInputDataToFilter;
        current_params.uiNumberOfInputDataItems = uiNumberOfInputDataItems;
    }

    double ConvertScaledNumToReal(int32_t ScaledNum, unsigned int ScaleFactor=16)
    {
        unsigned int Mask = 1 << (ScaleFactor - 1);

        Mask -= 1;

        unsigned int Mask0 = Mask ^ 0xffffffff;

        unsigned int WholeNum = (ScaledNum & Mask0) >> (ScaleFactor - 1);

        double WholeNumPart = WholeNum;

        unsigned int Frac = ScaledNum;

        Frac &= Mask;

        double FracPart = Frac;

        double denominator = pow(2.0, (double)(ScaleFactor - 1));

        FracPart *= (1.0 / denominator);

        double RealVal = WholeNumPart + FracPart;

        return RealVal;
    }

    bool GetScaledVal(double Value, int32_t & ScaledNum, uint32_t ScaleFactor = 16)
    {
        if (Value == 0.0)
        {
            ScaledNum = 0;

            return true;
        }
        double     accum = Value;

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
 //   void RunFilterForCheckingLowPass(FILTER_TEST_RESULTS & filterResults);

 //   void RunOUVFilter(FILTER_TEST_RESULTS & filterResults);

 //   void RunAdcFilter(FILTER_TEST_RESULTS & filterResults);

 //   void RunNeosFilter(FILTER_TEST_RESULTS & filterResults);

 //  void RunFixedPtFilter(FILTER_TEST_RESULTS & filterResults);

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
    bool DidApplyFilterPass(FILTER_TEST_RESULTS & ftrCompareTo, FILTER_TEST_RESULTS & ftrTargetBeingTested )
    {
        bool bPassed = true;
        
        uint32_t uiNumberOfInputs = sizeof(ftrCompareTo.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            if (!ftrTargetBeingTested.ftv[ui].bInTolerance)
            {
                bPassed = false;

                break;
            }
        }

        return bPassed;

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

        if (!OUVTest.IsFilteringReadyToStart()) return;

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            GetScaledVal(filterResults.ftv[ui].dInput, CurrentAtoDReading);

            bool bFilterApplied;

            OUVTest.ApplyFilter((CurrentAtoDReading >> 15), filterResults.uiCutoff, iOutput, bFilterApplied);

            if (bFilterApplied)
            {
                filterResults.ftv[ui].dOutput = ConvertScaledNumToReal(iOutput);
            }
        }

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

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / (sizeof(FILTER_TEST_VAL));

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            filterResults.ftv[ui].dOutput = dPrevOutput + (calcLagCoeff * (filterResults.ftv[ui].dInput - dPrevOutput));

            dPrevOutput = filterResults.ftv[ui].dOutput;
        }

    }

    void RunFixedPtFilter(FILTER_TEST_RESULTS & filterResults)
    {

        LowPassFilterFixedPt FixedPtTest(filterResults.uiCutoff,
            filterResults.uiPeriod,
            0,
            16);

        FixedPtTest.EnableFiltering();

        int32_t CurrentAtoDReading = 0;

        int32_t sOutput;

        bool    bFilterConfigured = false;

        FixedPtTest.ConfigureFilter(100, 50, bFilterConfigured);

        if (!bFilterConfigured) return;

        if (!FixedPtTest.IsFilteringReadyToStart()) return;

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            CurrentAtoDReading = filterResults.ftv[ui].dInput;

            bool bFilterApplied;

            FixedPtTest.ApplyFilter(CurrentAtoDReading, filterResults.uiCutoff, sOutput, bFilterApplied);

            if (bFilterApplied)
            {
                filterResults.ftv[ui].dOutput = ConvertScaledNumToReal(sOutput);
            }
        }
    }
    void RunFloatFilter(FILTER_TEST_RESULTS & filterResults)
    {

        LowPassFloat FloatTest(filterResults.uiCutoff,
            filterResults.uiPeriod,
            0.0);

        FloatTest.EnableFiltering();

        float CurrentAtoDReading = 0.0f;

        float fOutput;

        bool bFilterConfigured = false;

        FloatTest.ConfigureFilter(100, 50, bFilterConfigured);

        if (!bFilterConfigured) return;

        if (!FloatTest.IsFilteringReadyToStart()) return;

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            CurrentAtoDReading = filterResults.ftv[ui].dInput;

            bool bFilterApplied;

            FloatTest.ApplyFilter(CurrentAtoDReading, filterResults.uiCutoff, fOutput, bFilterApplied);

            if (bFilterApplied)
            {
                filterResults.ftv[ui].dOutput = fOutput;
            }
        }

    }
    void RunADCFilter(FILTER_TEST_RESULTS & filterResults)
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

        bool bFilterConfigured = false;

        ADCTest.ConfigureFilter(100, 50, bFilterConfigured);

        uint32_t uiNumberOfInputs = sizeof(filterResults.ftv) / sizeof(FILTER_TEST_VAL);

        for (uint32_t ui = 0; ui < uiNumberOfInputs; ui++)
        {
            GetScaledVal(filterResults.ftv[ui].dInput, CurrentAtoDReading);

            bool bFilterApplied;

            ADCTest.ApplyFilter((CurrentAtoDReading >> 15), filterResults.uiCutoff, iOutput, bFilterApplied);

            if (bFilterApplied)
            {
                filterResults.ftv[ui].dOutput = ConvertScaledNumToReal(iOutput);
            }
        }

    }
    bool BasicOUVFilterTest(FILTER_TEST_RESULTS &    ValidResults,
                            FILTER_TEST_RESULTS &    CalculatedResults,
                            FILTER_TEST_PARAMETERS & rftp )
    {
        double dLagCoeff = LagCoeff;

        InitFilterData(ValidResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunFilterForCheckingLowPass(ValidResults);

        InitFilterData(CalculatedResults, rftp.pdInputData, *rftp.pdInputData, rftp.uiCornerFreqHZ, rftp.uiSamplePeriodUS);

        InitFilterData(CalculatedResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunOUVFilter(CalculatedResults);

        CompareFilterOutputs(ValidResults, CalculatedResults, rftp.dActualCalcErrDiffThreshold);

        return DidApplyFilterPass(ValidResults, CalculatedResults);

    }
    bool BasicFixedPtFilterTest(FILTER_TEST_RESULTS &    ValidResults,
                                FILTER_TEST_RESULTS &    CalculatedResults,
                                FILTER_TEST_PARAMETERS & rftp)
    {
        InitFilterData(ValidResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunFilterForCheckingLowPass(ValidResults);

        InitFilterData(CalculatedResults, rftp.pdInputData, *rftp.pdInputData, rftp.uiCornerFreqHZ, rftp.uiSamplePeriodUS);

        InitFilterData(CalculatedResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunFixedPtFilter(CalculatedResults);

        CompareFilterOutputs(ValidResults, CalculatedResults, rftp.dActualCalcErrDiffThreshold);

        return DidApplyFilterPass(ValidResults, CalculatedResults);

    }
    bool BasicFloatFilterTest(FILTER_TEST_RESULTS &    ValidResults,
                              FILTER_TEST_RESULTS &    CalculatedResults,
                              FILTER_TEST_PARAMETERS & rftp)
    {
        InitFilterData(ValidResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunFilterForCheckingLowPass(ValidResults);

        InitFilterData(CalculatedResults, rftp.pdInputData, *rftp.pdInputData, rftp.uiCornerFreqHZ, rftp.uiSamplePeriodUS);

        InitFilterData(CalculatedResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunFloatFilter(CalculatedResults);

        CompareFilterOutputs(ValidResults, CalculatedResults, rftp.dActualCalcErrDiffThreshold);

        return DidApplyFilterPass(ValidResults, CalculatedResults);

    }
    bool BasicADCFilterTest(FILTER_TEST_RESULTS &    ValidResults,
                            FILTER_TEST_RESULTS &    CalculatedResults,
                            FILTER_TEST_PARAMETERS & rftp)
    {
        InitFilterData(ValidResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunFilterForCheckingLowPass(ValidResults);

        InitFilterData(CalculatedResults, rftp.pdInputData, *rftp.pdInputData, rftp.uiCornerFreqHZ, rftp.uiSamplePeriodUS);

        InitFilterData(CalculatedResults,
                       rftp.pdInputData,
                       *rftp.pdInputData,
                       rftp.uiCornerFreqHZ,
                       rftp.uiSamplePeriodUS);

        RunADCFilter(CalculatedResults);

        CompareFilterOutputs(ValidResults, CalculatedResults, rftp.dActualCalcErrDiffThreshold);

        return DidApplyFilterPass(ValidResults, CalculatedResults);

    }
 };

BOOST_AUTO_TEST_SUITE(test_LowPassFilter);

#define LOW_PASS_FILTER_TEST_CASE(name_) \
    BOOST_FIXTURE_TEST_CASE(LowPassFilter_##name_, LowPassFilter_fixture)

BOOST_AUTO_TEST_SUITE(test_LowPassFilter_InputFileSel);

// Testing valid input file selections
LOW_PASS_FILTER_TEST_CASE(FixedPt_BasicTest)
{
    BOOST_CHECK(
                  BasicFixedPtFilterTest(knownGoodFilterResults,
                                         FixedPtResults,
                                         default_params)
               );

}

LOW_PASS_FILTER_TEST_CASE(OUVFilter_BasicTest)
{
    BOOST_CHECK(
                 BasicOUVFilterTest(knownGoodFilterResults,
                                    OuvFilterResults,
                                    default_params)
               );

}

LOW_PASS_FILTER_TEST_CASE(FloatFilter_BasicTest)
{
    BOOST_CHECK(
                 BasicFloatFilterTest(knownGoodFilterResults,
                                      FloatFilterResults,
                                      default_params)
               );
}

LOW_PASS_FILTER_TEST_CASE(ADCFilter_BasicTest)
{
    BOOST_CHECK(
                 BasicFloatFilterTest(knownGoodFilterResults,
                                      ADCFilterResults,
                                      default_params)
               );
}

LOW_PASS_FILTER_TEST_CASE(InptFile_AbsltPath_NoKey_No_Whitespace)
{

}

LOW_PASS_FILTER_TEST_CASE(InptFile_AbsltPath_Key_No_Whitespace)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_AbsltPath_NoKey_Whitespace)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_AbsltPath_Key_Whitespace)
{
}


// Testing valid filter option selections
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_NoKey_NoW_FFiltAll)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_FFiltEvery)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_FFiltHFP)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_FFiltAS)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_FFiltBT)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_FFiltMap)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_FFiltUSB)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltAll)
{
    
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltEvery)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltHFP)
{

}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltAS)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltBT)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltMap)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_NoW_AFiltUSB)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltAll)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltEvery)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltHFP)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltAcnScrpt)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltBT)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltMap)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_FFiltUSB)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltAll)
{

}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltEvery)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltHFP)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltActnScrpt)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltBT)
{
}

LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltMap)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_Key_W_AFiltUSB)
{
}

// Testing valid output file selections
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_NoKey_NoW_Oabs)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_NoKey_NoW_Orel)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_NoKey_W_Oabs)
{
}
LOW_PASS_FILTER_TEST_CASE(InptFile_RelPath_NoKey_W_Orel)
{
}


LOW_PASS_FILTER_TEST_CASE(NotExistInptFile_RelPath_NoKey_No_Whitespace)
{
}
BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(test_LowPassFilter_OutputFileSel);
LOW_PASS_FILTER_TEST_CASE(NotExistOutptFile_RelPath_NoKey_No_Whitespace)
{
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()




