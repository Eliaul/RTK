//
// Created by 39894 on 2023/3/14.
//

#include "Satellite.h"

bool Satellite::operator<(const Satellite& s) const
{
    return naviSys == s.naviSys ? PRN < s.PRN : naviSys < s.naviSys;
}

bool Satellite::operator==(const Satellite& s) const
{
    return (naviSys == s.naviSys && PRN == s.PRN);
}