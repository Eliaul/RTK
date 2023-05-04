//
// Created by 39894 on 2023/3/15.
//

#include "SingleDifference.h"

void SingleDifferenceRange::DoSingleDifference(Range &range1, Range &range2, std::map<Satellite, BaseEphemeris*>& ephems)
{
    using enum SignalType;
    gpsTime = range1.gpsTime;
    for (auto& [sate, sateData1]: range1.sateDataMap)
    {
        if (!sateData1.isError && !sateData1.isLowHeightAngle && range2.sateDataMap.contains(sate) && !range2.sateDataMap[sate].isError && !range2.sateDataMap[sate].isLowHeightAngle && ephems[sate]->health != 1)
        {
            auto& sateData2 = range2.sateDataMap[sate];
            switch (sate.naviSys)
            {
                case NavigationSys::GPS:
                {
                    if (range1.sateDataMap[sate].obsMap[L1CA].parityKnownFlag == 1
                        && range1.sateDataMap[sate].obsMap[L2PY].parityKnownFlag == 1
                        && range2.sateDataMap[sate].obsMap[L1CA].parityKnownFlag == 1
                        && range2.sateDataMap[sate].obsMap[L2PY].parityKnownFlag == 1)
                    {
                        sdDataMap[sate].sdObsMap[L1CA].dP = sateData1.obsMap[L1CA].pseudorange - sateData2.obsMap[L1CA].pseudorange;
                        sdDataMap[sate].sdObsMap[L1CA].dL = sateData1.obsMap[L1CA].carrierPhase - sateData2.obsMap[L1CA].carrierPhase;
                        sdDataMap[sate].sdObsMap[L1CA].frequency = 1575.42e6;
                        sdDataMap[sate].sdObsMap[L2PY].dP = sateData1.obsMap[L2PY].pseudorange - sateData2.obsMap[L2PY].pseudorange;
                        sdDataMap[sate].sdObsMap[L2PY].dL = sateData1.obsMap[L2PY].carrierPhase - sateData2.obsMap[L2PY].carrierPhase;
                        sdDataMap[sate].sdObsMap[L2PY].frequency = 1227.60e6;
                    }
                    break;
                }
                case NavigationSys::BDS:
                {
                    if (range1.sateDataMap[sate].obsMap[B1I].parityKnownFlag == 1
                        && range1.sateDataMap[sate].obsMap[B3I].parityKnownFlag == 1
                        && range2.sateDataMap[sate].obsMap[B1I].parityKnownFlag == 1
                        && range2.sateDataMap[sate].obsMap[B3I].parityKnownFlag == 1)
                    {
                        sdDataMap[sate].sdObsMap[B1I].dP = sateData1.obsMap[B1I].pseudorange - sateData2.obsMap[B1I].pseudorange;
                        sdDataMap[sate].sdObsMap[B1I].dL = sateData1.obsMap[B1I].carrierPhase - sateData2.obsMap[B1I].carrierPhase;
                        sdDataMap[sate].sdObsMap[B1I].frequency = 1561.098e6;
                        sdDataMap[sate].sdObsMap[B3I].dP = sateData1.obsMap[B3I].pseudorange - sateData2.obsMap[B3I].pseudorange;
                        sdDataMap[sate].sdObsMap[B3I].dL = sateData1.obsMap[B3I].carrierPhase - sateData2.obsMap[B3I].carrierPhase;
                        sdDataMap[sate].sdObsMap[B3I].frequency = 1268.52e6;
                    }
                    break;
                }
                case NavigationSys::GLONASS:
                    break;
                case NavigationSys::QZSS:
                    break;
                case NavigationSys::SBAS:
                    break;
                case NavigationSys::Galileo:
                    break;
                case NavigationSys::Other:
                    break;
                case NavigationSys::NavIC:
                    break;
            }
        }
    }
}

void SingleDifferenceData::LinerCombination(SignalType signal1, SignalType signal2)
{
    auto carrierPhase1 = sdObsMap[signal1].dL;
    auto carrierPhase2 = sdObsMap[signal2].dL;
    auto frequency1 = sdObsMap[signal1].frequency;
    auto frequency2 = sdObsMap[signal2].frequency;
    auto pseudorange1 = sdObsMap[signal1].dP;
    auto pseudorange2 = sdObsMap[signal2].dP;
    mwComb = LIGHT_SPEED * (carrierPhase1 - carrierPhase2) / (frequency1 - frequency2)
             - (pseudorange1 * frequency1 + pseudorange2 * frequency2) / (frequency1 + frequency2);
    mwCombAve = mwComb;
    gfComb = LIGHT_SPEED * (carrierPhase1 / frequency1 - carrierPhase2 / frequency2);
    //ifComb = (frequency1 * frequency1 * pseudorange1 - frequency2 * frequency2 * pseudorange2) / (pow(frequency1, 2) - pow(frequency2, 2));
}
