//
// Created by 39894 on 2023/3/14.
//

#pragma once

#pragma warning(disable:26495)

#include <map>
#include <iostream>
#include "../TimeSystem/GPSTime.h"
#include "Satellite.h"
#include "SatelliteData.h"
#include "BaseEphemeris.h"

class Range
{
public:
    GPSTime gpsTime;
    unsigned long obsNum;
    /// <summary>
    /// 卫星与卫星数据字典
    /// </summary>
    std::map<Satellite, SatelliteData> sateDataMap;

    Range() = default;

    /// <summary>
    /// 计算所有卫星的位置,速度,钟差等数据
    /// </summary>
    /// <param name="ephems">星历字典</param>
    void GetSateMotionState(std::map<Satellite, BaseEphemeris*>& ephems);

};

std::ostream& operator<<(std::ostream& basicOstream, const Range& range);