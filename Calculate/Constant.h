//
// Created by 39894 on 2023/3/14.
//

#pragma once

#include <numbers>

#define DEG2RAD(x) ((x) * (std::numbers::pi / 180))
#define RAD2DEG(x) ((x) * (180 / std::numbers::pi))

constexpr double LIGHT_SPEED = 2.99792458e8;
constexpr double H0 = 0; //米
constexpr double T0 = 288.16; //开尔文
constexpr double P0 = 1013.25; //mbar
constexpr double RH0 = 0.5; //%