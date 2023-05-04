//
// Created by 39894 on 2023/3/20.
//

#pragma once

#include "DataStructure/PositionResult.hpp"
#include "SingleDifference.h"
#include "DataStructure/Range.h"
#include "CoordinateSystem/XYZCoordinate.hpp"
#include "DoubleDifference.h"
#include "DataStructure/BestPos.h"


class FloatSolution
{
public:
    constexpr static double sigma[] = {0.5,0.5,0.0005,0.0005};
    XYZCoordinate baseVector;
    XYZCoordinate roverPos;
    Vec estimatedArgs;
    Matrix Q;
    GPSTime time;
    Vec ambiguousArgs;
    Matrix ambiguousQ;
    bool isAvailable;
    std::map<Satellite, double> floatAmbiguity;
    std::map<NavigationSys, int> sateNumMap;
    double sigma_;
    double sigma_x;
    double sigma_y;
    double sigma_z;

    template<CoordinateSystem T>
    void RelativePositioning(SingleDifferenceRange& sdRange, DoubleDifferenceOBS& ddOBS, Range& roverRange, Range& baseRange, BestPos& baseBestPos, SPPResult<T>& roverSPP);

    std::vector<Satellite> sateArray;
};

template<CoordinateSystem T>
void FloatSolution::RelativePositioning(SingleDifferenceRange &sdRange, DoubleDifferenceOBS& ddOBS, Range &roverRange, Range& baseRange, BestPos& baseBestPos, SPPResult<T> &roverSPP)
{
    isAvailable = false;
    roverPos = roverSPP.posXYZ;
    sateNumMap[NavigationSys::GPS] = std::count_if(sdRange.sdDataMap.begin(), sdRange.sdDataMap.end(), [&](const auto& item) {
        return item.first.naviSys == NavigationSys::GPS && !item.second.isError;
    });
    sateNumMap[NavigationSys::BDS] = std::count_if(sdRange.sdDataMap.begin(), sdRange.sdDataMap.end(), [&](const auto& item) {
        return item.first.naviSys == NavigationSys::BDS && !item.second.isError;
    });
    auto basePos = baseBestPos.xyzPos;
    auto sateNum = sateNumMap[NavigationSys::GPS] + sateNumMap[NavigationSys::BDS];
    if (sateNum <= 4)
    {
        return;
    }
    auto refSateNum = ddOBS.refSatellite.size();
    Matrix B = Matrix::Zeros(4 * (sateNum - refSateNum), 2 * (sateNum - refSateNum) + 3);
    Vec w(4 * (sateNum - refSateNum));
    Matrix P = Matrix::Zeros(4 * (sateNum - refSateNum), 4 * (sateNum - refSateNum));
    estimatedArgs = Vec(2 * (sateNum - refSateNum) + 3);
    estimatedArgs(1) = roverPos(1);
    estimatedArgs(2) = roverPos(2);
    estimatedArgs(3) = roverPos(3);
    int bRow = 1;
    int bCol = 4;
    int wRow = 1;
    int pRow = 1;
    int itNum = 0;
    sateArray = std::vector<Satellite>(sateNum - refSateNum);
    while (true)
    {
        NavigationSys nsLast = sdRange.sdDataMap.begin()->first.naviSys;
        int nLast = 0;
        int idx = 0;
        itNum++;
        int sateIdx = 0;
        for (auto& [sate, sdData]: sdRange.sdDataMap)
        {
            if (sdData.isError || sate == ddOBS.refSatellite[sate.naviSys])
                continue;
            sateArray[sateIdx] = sate;
            sateIdx++;
            auto baseSatePos = baseRange.sateDataMap[sate].satePos_Rz;
            auto baseRefSatePos = baseRange.sateDataMap[ddOBS.refSatellite[sate.naviSys]].satePos_Rz;
            auto roverSatePos = roverRange.sateDataMap[sate].satePos_Rz;
            auto roverRefSatePos = roverRange.sateDataMap[ddOBS.refSatellite[sate.naviSys]].satePos_Rz;
            auto signal1 = GetSignalType(sate.naviSys, 1);
            auto signal2 = GetSignalType(sate.naviSys, 2);
            auto lambda1 = LIGHT_SPEED / roverRange.sateDataMap[sate].obsMap[signal1].frequency;
            auto lambda2 = LIGHT_SPEED / roverRange.sateDataMap[sate].obsMap[signal2].frequency;
            auto disRoverToSate = XYZCoordinate::Distance(roverPos, roverSatePos);
            auto disRoverToRef = XYZCoordinate::Distance(roverPos, roverRefSatePos);
            auto disBaseToSate = XYZCoordinate::Distance(basePos, baseSatePos);
            auto disBaseToRef = XYZCoordinate::Distance(basePos, baseRefSatePos);
            auto doubleDifferenceDis = disRoverToSate - disRoverToRef - disBaseToSate + disBaseToRef;
            auto doubleDifferenceP1 = sdRange.sdDataMap[sate].sdObsMap[signal1].dP - sdRange.sdDataMap[ddOBS.refSatellite[sate.naviSys]].sdObsMap[signal1].dP;
            auto doubleDifferenceL1 = (sdRange.sdDataMap[sate].sdObsMap[signal1].dL - sdRange.sdDataMap[ddOBS.refSatellite[sate.naviSys]].sdObsMap[signal1].dL) * lambda1;
            auto doubleDifferenceP2 = sdRange.sdDataMap[sate].sdObsMap[signal2].dP - sdRange.sdDataMap[ddOBS.refSatellite[sate.naviSys]].sdObsMap[signal2].dP;
            auto doubleDifferenceL2 = (sdRange.sdDataMap[sate].sdObsMap[signal2].dL - sdRange.sdDataMap[ddOBS.refSatellite[sate.naviSys]].sdObsMap[signal2].dL) * lambda2;
            auto n = ddOBS.refSatellite.contains(sate.naviSys) ? sateNumMap[sate.naviSys] - 1 : sateNumMap[sate.naviSys];
            for (int i = 0; i < 4; i++, bRow++)
            {
                for (int j = 1; j < 4; j++)
                {
                    B(bRow, j) = (roverPos(j) - roverSatePos(j)) / disRoverToSate
                                 - (roverPos(j) - roverRefSatePos(j)) / disRoverToRef;
                }
                if (i == 2)
                {
                    B(bRow, bCol) = lambda1;
                    bCol++;
                    B(bRow + 1, bCol) = lambda2;
                    bCol++;
                }
            }
            w(wRow) = doubleDifferenceP1 - doubleDifferenceDis;
            wRow++;
            w(wRow) = doubleDifferenceP2 - doubleDifferenceDis;
            wRow++;
            if (itNum == 1)
            {
                estimatedArgs(3 + (wRow - 1) / 2) = (doubleDifferenceL1 - doubleDifferenceP1) / lambda1;
            }
            w(wRow) = doubleDifferenceL1 - doubleDifferenceDis - lambda1 * estimatedArgs(3 + (wRow - 1) / 2);
            wRow++;
            if (itNum == 1)
            {
                estimatedArgs(3 + wRow / 2) = (doubleDifferenceL2 - doubleDifferenceP2) / lambda2;
            }
            w(wRow) = doubleDifferenceL2 - doubleDifferenceDis - lambda2 * estimatedArgs(3 + wRow / 2);
            wRow++;
            if (sate.naviSys != nsLast)
            {
                idx += 4 * nLast;
            }
            for (int i = 0; i < 4; i++)
            {
                for (int j = idx + i; j < idx + 4 * n + i; j+=4)
                {
                    P(pRow + i, j + 1) = (j + 1 == pRow + i) ? 1 / (2 * sigma[i]) * (double)n / (n + 1) : -1 / (2 * sigma[i]) * 1.0 / (n + 1);
                }
            }
            pRow+=4;
            nsLast = sate.naviSys;
            nLast = n;
        }
        Q = Matrix::Inverse(Matrix::Transpose(B) * P * B);
        auto x = Q * B.Transpose() * P * w;
        estimatedArgs += x;
        roverPos(1) = estimatedArgs(1);
        roverPos(2) = estimatedArgs(2);
        roverPos(3) = estimatedArgs(3);
        bRow = 1;
        bCol = 4;
        wRow = 1;
        pRow = 1;
        if (x.Norm() < 1e-7)
        {
            isAvailable = true;
            ambiguousArgs = Vec(estimatedArgs.Length() - 3);
            for (int i = 0; i < ambiguousArgs.Length(); i++)
            {
                ambiguousArgs(i + 1) = estimatedArgs(i + 4);
                floatAmbiguity[sateArray[i / 2]] = ambiguousArgs(i + 1);
            }
            time = roverRange.gpsTime;
            ambiguousQ = Matrix(Q.Row() - 3, Q.Col() - 3);
            baseVector = roverPos - baseBestPos.xyzPos;
            for (int i = 0; i < ambiguousQ.Row(); i++)
                for (int j = 0; j < ambiguousQ.Col(); j++)
                    ambiguousQ(i + 1, j + 1) = Q(i + 4, j + 4);
            Matrix V = w - B * x;
            sigma_ = std::sqrt((V.Transpose() * P * V)(1, 1) / (B.Row() - B.Col()));
            sigma_x = std::sqrt(sigma_ * sigma_ * Q(1, 1));
            sigma_y = std::sqrt(sigma_ * sigma_ * Q(2, 2));
            sigma_z = std::sqrt(sigma_ * sigma_ * Q(3, 3));
            break;
        }
        if (itNum > 10)
        {
            isAvailable = false;
            break;
        }
    }

}
