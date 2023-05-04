//
// Created by 39894 on 2023/4/3.
//

#include "FixedSolution.h"

void FixedSolution::GetFixedSolution(SingleDifferenceRange &sdRange, DoubleDifferenceOBS &ddOBS, Range& roverRange, Range& baseRange, BestPos& baseBestPos, Matrix &F,
                                     FloatSolution &floatSolution)
{
    isAvailable = false;
    roverPos = floatSolution.roverPos;
    for (int i = 0; i < F.Row(); i++)
        fixedAmbiguity[floatSolution.sateArray[i / 2]][i % 2] = F(i + 1, 1);
    auto basePos = baseBestPos.xyzPos;
    auto sateNum = floatSolution.sateNumMap[NavigationSys::GPS] + floatSolution.sateNumMap[NavigationSys::BDS];
    auto refSateNum = ddOBS.refSatellite.size();
    Matrix B = Matrix::Zeros(2 * (sateNum - refSateNum), 3);
    Vec w(2 * (sateNum - refSateNum));
    Matrix P = Matrix::Zeros(2 * (sateNum - refSateNum), 2 * (sateNum - refSateNum));
    estimatedArgs = Vec(3);
    estimatedArgs(1) = roverPos(1);
    estimatedArgs(2) = roverPos(2);
    estimatedArgs(3) = roverPos(3);
    int bRow = 1;
    int wRow = 1;
    int pRow = 1;
    int itNum = 0;
    while (true)
    {
        NavigationSys nsLast = sdRange.sdDataMap.begin()->first.naviSys;
        int nLast = 0;
        int idx = 0;
        itNum++;
        for (auto& [sate, sdData]: sdRange.sdDataMap)
        {
            if (sdData.isError || sate == ddOBS.refSatellite[sate.naviSys])
                continue;
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
            auto doubleDifferenceL1 = (sdRange.sdDataMap[sate].sdObsMap[signal1].dL - sdRange.sdDataMap[ddOBS.refSatellite[sate.naviSys]].sdObsMap[signal1].dL) * lambda1;
            auto doubleDifferenceL2 = (sdRange.sdDataMap[sate].sdObsMap[signal2].dL - sdRange.sdDataMap[ddOBS.refSatellite[sate.naviSys]].sdObsMap[signal2].dL) * lambda2;
            auto n = ddOBS.refSatellite.contains(sate.naviSys) ? floatSolution.sateNumMap[sate.naviSys] - 1 : floatSolution.sateNumMap[sate.naviSys];
            for (int i = 0; i < 2; i++, bRow++)
            {
                for (int j = 1; j < 4; j++)
                {
                    B(bRow, j) = (roverPos(j) - roverSatePos(j)) / disRoverToSate
                                 - (roverPos(j) - roverRefSatePos(j)) / disRoverToRef;
                }
            }
            w(wRow) = doubleDifferenceL1 - doubleDifferenceDis - lambda1 * fixedAmbiguity[sate][0];
            wRow++;
            w(wRow) = doubleDifferenceL2 - doubleDifferenceDis - lambda2 * fixedAmbiguity[sate][1];
            wRow++;
            if (sate.naviSys != nsLast)
            {
                idx += 2 * nLast;
            }
            for (int i = 0; i < 2; i++)
            {
                for (int j = idx + i; j < idx + 2 * n + i; j+=2)
                {
                    P(pRow + i, j + 1) = (j + 1 == pRow + i) ? 1 / (1) * (double)n / (n + 1) : -1 / (1) * 1.0 / (n + 1);
                }
            }
            pRow+=2;
            nsLast = sate.naviSys;
            nLast = n;
        }
        Matrix Q = Matrix::Inverse(Matrix::Transpose(B) * P * B);
        auto x = Q * B.Transpose() * P * w;
        estimatedArgs += x;
        roverPos(1) = estimatedArgs(1);
        roverPos(2) = estimatedArgs(2);
        roverPos(3) = estimatedArgs(3);
        bRow = 1;
        wRow = 1;
        pRow = 1;
        if (x.Norm() < 1e-7)
        {
            time = roverRange.gpsTime;
            isAvailable = true;
            baseVector = roverPos - basePos;
            Matrix V = w - B * x;
            sigma = std::sqrt((V.Transpose() * P * V)(1, 1) / (B.Row() - B.Col()));
            sigma_xx = std::sqrt(sigma * sigma * Q(1, 1));
            sigma_yy = std::sqrt(sigma * sigma * Q(2, 2));
            sigma_zz = std::sqrt(sigma * sigma * Q(3, 3));
            roverPosBLH = XYZCoordinate::ToBLH<CoordinateSystem::WGS84>(roverPos);
            double sinL = std::sin(roverPosBLH(2) * M_PI / 180);
            double cosL = std::cos(roverPosBLH(2) * M_PI / 180);
            double sinB = std::sin(roverPosBLH(1) * M_PI / 180);
            double cosB = std::cos(roverPosBLH(1) * M_PI / 180);
            Matrix H {{-sinL, cosL, 0}, {-sinB * cosL, -sinB * sinL, cosB}, {-cosB * cosL, cosB* sinL, sinB}};
            Matrix Qxyz = {{Q(1,1), Q(1,2), Q(1,3)}, {Q(2,1), Q(2,2), Q(2,3)}, {Q(3,1), Q(3,2), Q(3,3)}};
            Matrix Qenu = H * Qxyz * H.Transpose();
            PDOP = std::sqrt(Qenu(1, 1) + Qenu(2, 2) + Qenu(3, 3));
            VDOP = std::sqrt(Qenu(3, 3));
            HDOP = std::sqrt(Qenu(1, 1) + Qenu(2, 2));
            break;
        }
        if (itNum > 10)
        {
            isAvailable = false;
            break;
        }
    }
}
