#ifndef LOWPASS_OFFLINE_TESTING
#define LOWPASS_OFFLINE_TESTING
#include "OUVFilter.hpp"
#include "ADCFilter.hpp"
#include "LowPassFloat.hpp"
#include "LowPassFiltersFixedPt.hpp"

namespace LowPassOfflineTesting
{
    using namespace LowPassFilters;
    //static OUVFilter  OUV(100, 50, 0x1fa, 16);

	static const int x = 0;

    static const float PI = 3.1415927f;

    static const float Cutoff = 100.0f;

    static const float Period = .000001f * 50.0f;

    static const float fRadiansTC = 2.0f * PI * Cutoff * Period;

    static const float LagCoeff = fRadiansTC / (fRadiansTC + 2.0f);

    static const unsigned int NUMBER_OF_FILTER_TEST_VALS = 10;

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

	static FILTER_TEST_RESULTS   AdcFilterResults;

	static FILTER_TEST_RESULTS   NeosFilterResults;

    static const double SampleInputSteadState[NUMBER_OF_FILTER_TEST_VALS] = { 10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0 };

    static const double SampleInputRamp[NUMBER_OF_FILTER_TEST_VALS] = { 1.0,2.0,4.0,8.0,16.0,32.0,64.0,128.0,256.0,512.0 };

    static const double * pSteadyStateSample = SampleInputSteadState;

    static const double * pRampSample = SampleInputRamp;

    bool GetScaledVal(double Value, int32_t & ScaledNum, uint32_t ScaleFactor = 16);

	double ConvertScaledNumToReal(int32_t ScaledNum, unsigned int ScaleFactor = 16);

    void InitFilterData(FILTER_TEST_RESULTS & filterResults, double const * & dInputVals, double dInitialVal, uint32_t Cutoff, uint32_t Period);

    void RunFilterForCheckingLowPass(FILTER_TEST_RESULTS & filterResults);

    void RunOUVFilter(FILTER_TEST_RESULTS & filterResults);

	void RunAdcFilter(FILTER_TEST_RESULTS & filterResults);

	void RunNeosFilter(FILTER_TEST_RESULTS & filterResults);
	
	void RunFixedPtFilter(FILTER_TEST_RESULTS & filterResults);

	void CompareFilterOutputs(FILTER_TEST_RESULTS & ftrCompareTo, FILTER_TEST_RESULTS & ftrTargetBeingTested, double tolerancePercentAllowed);
};
#endif


