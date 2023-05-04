//
// Created by 39894 on 2023/3/18.
//

#include "DoubleDifference.h"

void DoubleDifferenceOBS::DetermineRefSate(SingleDifferenceRange &sdRange, Range &roverRange, Range &baseRange)
{
    float gpsMaxCN0 = 0;
    float bdsMaxCN0 = 0;
    using enum NavigationSys;
    std::map<NavigationSys, float> maxCN0 = {
        {GPS, 0}, {BDS, 0}
    };
    for (auto& [sate, sdData]: sdRange.sdDataMap)
    {
        if (!sdData.isError)
        {
            auto signal1 = GetSignalType(sate.naviSys, 1);
            auto signal2 = GetSignalType(sate.naviSys, 2);
            auto sig1cn0 = roverRange.sateDataMap[sate].obsMap[signal1].carrierToNoiseDensityRatio;
            auto sig2cn0 = roverRange.sateDataMap[sate].obsMap[signal2].carrierToNoiseDensityRatio;
            if (sig1cn0 > 40 && sig2cn0 > 28 && sig1cn0 > maxCN0[sate.naviSys])
            {
                maxCN0[sate.naviSys] = sig1cn0;
                refSatellite[sate.naviSys] = sate;
            }
//            if (sate.naviSys == NavigationSys::GPS
//                && roverRange.sateDataMap[sate].obsMap[SignalType::L1CA].carrierToNoiseDensityRatio > 40
//                && roverRange.sateDataMap[sate].obsMap[SignalType::L2PY].carrierToNoiseDensityRatio > 28)
//            {
//                if (roverRange.sateDataMap[sate].obsMap[SignalType::L1CA].carrierToNoiseDensityRatio > gpsMaxCN0)
//                {
//                    gpsMaxCN0 = roverRange.sateDataMap[sate].obsMap[SignalType::L1CA].carrierToNoiseDensityRatio;
//                    refSatellite[NavigationSys::GPS] = sate;
//                }
//            }
//            if (sate.naviSys == NavigationSys::BDS
//                && roverRange.sateDataMap[sate].obsMap[SignalType::B1I].carrierToNoiseDensityRatio > 40
//                && roverRange.sateDataMap[sate].obsMap[SignalType::B3I].carrierToNoiseDensityRatio > 28)
//            {
//                if (roverRange.sateDataMap[sate].obsMap[SignalType::B1I].carrierToNoiseDensityRatio > bdsMaxCN0)
//                {
//                    bdsMaxCN0 = roverRange.sateDataMap[sate].obsMap[SignalType::B1I].carrierToNoiseDensityRatio;
//                    refSatellite[NavigationSys::BDS] = sate;
//                }
//            }
        }
    }
}
