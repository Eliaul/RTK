//
// Created by 39894 on 2023/4/10.
//


#include "Export.h"
#include <format>

using namespace std;
using enum NavigationSys;

std::string ExportCSV(FixedSolution &fixedSolution, DoubleDifferenceOBS& ddOBS, FloatSolution& floatSolution, int rtk, int total)
{
    auto week = fixedSolution.time.GpsWeek().count();
    auto sec = fixedSolution.time.GpsSecOfWeek().count();
    auto& baseVec = fixedSolution.baseVector;
    auto& roverXYZ = fixedSolution.roverPos;
    auto& roverBLH  = fixedSolution.roverPosBLH;
    auto ratio = fixedSolution.ratio;
    auto sigma = fixedSolution.sigma;
    auto sigma_xx = fixedSolution.sigma_xx;
    auto sigma_yy = fixedSolution.sigma_yy;
    auto sigma_zz = fixedSolution.sigma_zz;
    auto pdop = fixedSolution.PDOP;
    auto hdop = fixedSolution.HDOP;
    auto vdop = fixedSolution.VDOP;
    auto& sateNumMap = floatSolution.sateNumMap;
    auto gpsNum = sateNumMap.contains(GPS) ? sateNumMap[GPS] : 0; //包括参考星
    auto bdsNum = sateNumMap.contains(BDS) ? sateNumMap[BDS] : 0; //同上
    auto& refMap = ddOBS.refSatellite;
    auto gpsRefPRN = refMap.contains(GPS) ? refMap[GPS].PRN : -1;
    auto bdsRefPRN = refMap.contains(BDS) ? refMap[BDS].PRN : -1;
    return format(
                "{},{},{},{},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.9f},{:13.9f},{:8.4f},{:9.3f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{},{},{},{}",
                rtk, total,
                week, sec,
                baseVec(1), baseVec(2), baseVec(3),
                roverXYZ(1), roverXYZ(2), roverXYZ(3),
                roverBLH(1), roverBLH(2), roverBLH(3),
                ratio,
                sigma, sigma_xx, sigma_yy, sigma_zz,pdop,
                vdop, hdop,
                gpsRefPRN, bdsRefPRN, gpsNum, bdsNum
            );
}

std::string ExportCSV(FixedSolution &fixedSolution, DoubleDifferenceOBS& ddOBS, FloatSolution& floatSolution)
{
    auto week = fixedSolution.time.GpsWeek().count();
    auto sec = fixedSolution.time.GpsSecOfWeek().count();
    auto& baseVec = fixedSolution.baseVector;
    auto& roverXYZ = fixedSolution.roverPos;
    auto& roverBLH  = fixedSolution.roverPosBLH;
    auto ratio = fixedSolution.ratio;
    auto sigma = fixedSolution.sigma;
    auto sigma_xx = fixedSolution.sigma_xx;
    auto sigma_yy = fixedSolution.sigma_yy;
    auto sigma_zz = fixedSolution.sigma_zz;
    auto pdop = fixedSolution.PDOP;
    auto hdop = fixedSolution.HDOP;
    auto vdop = fixedSolution.VDOP;
    auto& sateNumMap = floatSolution.sateNumMap;
    auto gpsNum = sateNumMap.contains(GPS) ? sateNumMap[GPS] : 0; //包括参考星
    auto bdsNum = sateNumMap.contains(BDS) ? sateNumMap[BDS] : 0; //同上
    auto& refMap = ddOBS.refSatellite;
    auto gpsRefPRN = refMap.contains(GPS) ? refMap[GPS].PRN : -1;
    auto bdsRefPRN = refMap.contains(BDS) ? refMap[BDS].PRN : -1;
    return format(
            "{},{},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.9f},{:13.9f},{:8.4f},{:9.3f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{:10.6f},{},{},{},{}",
            week, sec,
            baseVec(1), baseVec(2), baseVec(3),
            roverXYZ(1), roverXYZ(2), roverXYZ(3),
            roverBLH(1), roverBLH(2), roverBLH(3),
            ratio,
            sigma, sigma_xx, sigma_yy, sigma_zz,pdop,
            vdop, hdop,
            gpsRefPRN, bdsRefPRN, gpsNum, bdsNum
    );
}

std::string ExportCSV(FloatSolution& floatSolution)
{
    auto week = floatSolution.time.GpsWeek().count();
    auto sec = floatSolution.time.GpsSecOfWeek().count();
    auto& roverPos = floatSolution.roverPos;
    auto& baseline= floatSolution.baseVector;
    auto sigma = floatSolution.sigma_;
    auto sigma_x = floatSolution.sigma_x;
    auto sigma_y = floatSolution.sigma_y;
    auto sigma_z = floatSolution.sigma_z;
    return format(
            "{},{},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:13.4f},{:10.6f}",
            week, sec,
            baseline(1), baseline(2), baseline(3),
            roverPos(1), roverPos(2), roverPos(3),
            sigma
    );
}