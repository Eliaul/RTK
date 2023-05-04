//
// Created by 39894 on 2023/4/18.
//

#pragma once

#include <string>
#include "CoordinateSystem/XYZCoordinate.hpp"

void WebRTK(
    const char* roverIP, const unsigned short roverPort,
    const char* baseIP, const unsigned short basePort
);

void FileRTK(std::string roverPath, std::string basePath, XYZCoordinate coordinate);
