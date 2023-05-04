//
// Created by 39894 on 2023/4/6.
//

#ifndef RTK2_CONFIG_H
#define RTK2_CONFIG_H
#include "minIni.h"
#include <optional>
#include "../CoordinateSystem/XYZCoordinate.hpp"

struct SPPConfig
{
    std::optional<std::string> obsFile;
};

struct RTKConfig
{
    std::optional<std::string> roverObsFile;
    std::optional<std::string> baseObsFile;
    short webOrFile;
    char* baseIP;
    char* roverIP;
    XYZCoordinate basePos;
    unsigned short basePort;
    unsigned short roverPort;
};

struct Config
{
    SPPConfig sppConfig;
    RTKConfig rtkConfig;
    short mode;
    void Read(const char* configFile);
};


#endif //RTK2_CONFIG_H
