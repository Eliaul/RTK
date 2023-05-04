//
// Created by 39894 on 2023/3/15.
//

#pragma once

#include "TimeSystem/GPSTime.h"
#include "DataStructure/Satellite.h"
#include "DataStructure/Range.h"
#include <map>

struct SingleDifferenceOBS
{
    double dP;
    double dL;
    double frequency;
};

class SingleDifferenceData
{
public:
    double mwComb;
    double gfComb;
    double mwCombAve;
    int epochIndex;
    bool isError;
    std::map<SignalType, SingleDifferenceOBS> sdObsMap;

    void LinerCombination(SignalType signal1, SignalType signal2);
    SingleDifferenceData(): mwComb(0), gfComb(0), mwCombAve(0), epochIndex(0), isError(true) {}
};

class SingleDifferenceRange
{
public:
    GPSTime gpsTime;
    std::map<Satellite, SingleDifferenceData> sdDataMap;

    void DoSingleDifference(Range& range1, Range& range2, std::map<Satellite, BaseEphemeris*>& ephems);
};