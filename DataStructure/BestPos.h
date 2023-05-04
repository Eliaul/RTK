//
// Created by 39894 on 2023/3/20.
//

#pragma once

#include "CoordinateSystem/XYZCoordinate.hpp"

class BestPos
{
public:
    double latitude;
    double longitude;
    double height;

    XYZCoordinate xyzPos;

    BestPos() = default;
    BestPos(double lat, double lon, double h): latitude(lat), longitude(lon), height(h)
    {
        xyzPos = XYZCoordinate::FromBLH(BLHCoordinate<CoordinateSystem::WGS84>(lat, lon, h));
    }
};