//
// Created by 39894 on 2023/3/18.
//

#pragma once

#include "DataStructure/Satellite.h"
#include "DataStructure/Range.h"

template<class T>
class ErrorDetector;

template<>
class ErrorDetector<Range>
{
public:
    ErrorDetector(): preRange() { }
    void ErrorDetect(Range& curRange);
private:
    Range preRange;
};

template<>
class ErrorDetector<SingleDifferenceRange>
{
public:
    ErrorDetector(): preSDRange() { }
    void ErrorDetect(SingleDifferenceRange& curSDRange);
private:
    SingleDifferenceRange preSDRange;
};

inline void ErrorDetector<Range>::ErrorDetect(Range& curRange)
{
    using enum SignalType;
    for (auto it = curRange.sateDataMap.begin(); it != curRange.sateDataMap.end(); )
    {
        if (it->second.obsMap.size() > 1)
        {
            it->second.isError = true;
            SignalType fr1 = it->first.naviSys == NavigationSys::GPS ? L1CA : B1I;
            SignalType fr2 = it->first.naviSys == NavigationSys::GPS ? L2PY : B3I;
            it->second.LinerCombination(fr1, fr2);
            if (preRange.sateDataMap.contains(it->first) && (curRange.gpsTime - preRange.gpsTime).count() < 1.1e9)
            {
                if (abs(it->second.mwComb - preRange.sateDataMap[it->first].mwCombAve) < 1.5 && abs(it->second.gfComb - preRange.sateDataMap[it->first].gfComb) < 0.05)
                {
                    //此时表明该历元观测值可用
                    auto i = preRange.sateDataMap[it->first].epochIndex;
                    it->second.epochIndex = i + 1;
                    it->second.mwCombAve = (i - 1) * preRange.sateDataMap[it->first].mwCombAve / ((double)i) + it->second.mwComb / ((double)i);
                    it->second.isError = false;
                }
                else
                {
                    it->second.epochIndex = 1; //此时表明该历元观测值有粗差或者周跳
                }
            }
            else
            {
                it->second.epochIndex = 1; //此时表明观测值有中断
            }
            ++it;
        }
        else
        {
            it = curRange.sateDataMap.erase(it);  //没有双频观测值,删除该卫星
        }
    }
    preRange = curRange;
}

inline void ErrorDetector<SingleDifferenceRange>::ErrorDetect(SingleDifferenceRange &curSDRange)
{
    using enum SignalType;
    for (auto it = curSDRange.sdDataMap.begin(); it != curSDRange.sdDataMap.end(); )
    {
        if (it->second.sdObsMap.size() > 1)
        {
            it->second.isError = true;
            SignalType fr1 = it->first.naviSys == NavigationSys::GPS ? L1CA : B1I;
            SignalType fr2 = it->first.naviSys == NavigationSys::GPS ? L2PY : B3I;
            it->second.LinerCombination(fr1, fr2);
            if (preSDRange.sdDataMap.contains(it->first) && (curSDRange.gpsTime - preSDRange.gpsTime).count() < 1.1e9)
            {
                if (abs(it->second.mwComb - preSDRange.sdDataMap[it->first].mwCombAve) < 1.5 && abs(it->second.gfComb - preSDRange.sdDataMap[it->first].gfComb) < 0.05)
                {
                    //此时表明该历元观测值可用
                    auto i = preSDRange.sdDataMap[it->first].epochIndex;
                    it->second.epochIndex = i + 1;
                    it->second.mwCombAve = (i - 1) * preSDRange.sdDataMap[it->first].mwCombAve / ((double)i) + it->second.mwComb / ((double)i);
                    it->second.isError = false;
                }
                else
                {
                    it->second.epochIndex = 1; //此时表明该历元观测值有粗差或者周跳
                }
            }
            else
            {
                it->second.epochIndex = 1; //此时表明观测值有中断
            }
            ++it;
        }
        else
        {
            it = curSDRange.sdDataMap.erase(it);  //没有双频观测值,删除该卫星
        }
    }
    preSDRange = curSDRange;
}
