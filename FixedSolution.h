//
// Created by 39894 on 2023/3/30.
//

#pragma once

#include "CoordinateSystem/XYZCoordinate.hpp"
#include "SingleDifference.h"
#include "DoubleDifference.h"
#include "RelativePositioning.hpp"

class FixedSolution
{
public:
    XYZCoordinate roverPos;
    BLHCoordinate<CoordinateSystem::WGS84> roverPosBLH;
    XYZCoordinate baseVector;
    bool isAvailable;
    Vec estimatedArgs;
    double sigma;
    double sigma_xx;
    double sigma_yy;
    double sigma_zz;
    double ratio;
    double PDOP;
    double VDOP;
    double HDOP;
    GPSTime time;
    std::map<Satellite, double[2]> fixedAmbiguity;

    void GetFixedSolution(SingleDifferenceRange& sdRange, DoubleDifferenceOBS& ddOBS, Range& roverRange, Range& baseRange, BestPos& baseBestPos, Matrix& F, FloatSolution& floatSolution);
};

inline std::ostream& operator<<(std::ostream& out, FixedSolution& fixedSolution)
{
    if (fixedSolution.isAvailable)
    {
        out << fixedSolution.time << std::format(" ratio:{:8.3f} X:{:12.4f} Y:{:12.4f} Z:{:12.4f} "
                                                       "dX:{:12.4f} dY:{:12.4f} dZ:{:12.4f} "
                                                       "B:{:12.8f} L:{:12.8f} H:{:8.3f} PDOP:{:8.3f} "
                                                       "Sigma:{:8.4f}\n",
                                                       fixedSolution.ratio, fixedSolution.roverPos(1),
                                                       fixedSolution.roverPos(2), fixedSolution.roverPos(3),
                                                       fixedSolution.baseVector(1), fixedSolution.baseVector(2),
                                                       fixedSolution.baseVector(3), fixedSolution.roverPosBLH(1),
                                                       fixedSolution.roverPosBLH(2), fixedSolution.roverPosBLH(3),
                                                       fixedSolution.PDOP, fixedSolution.sigma);
        return out;
    }
}