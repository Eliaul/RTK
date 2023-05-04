//
// Created by 39894 on 2023/3/18.
//

#pragma once

#include "DataStructure/Satellite.h"
#include "SingleDifference.h"
#include <map>

class DoubleDifferenceOBS
{
public:
    std::map<NavigationSys, Satellite> refSatellite;

    void DetermineRefSate(SingleDifferenceRange& sdRange, Range& roverRange, Range& baseRange);
};