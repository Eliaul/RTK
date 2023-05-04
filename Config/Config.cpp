//
// Created by 39894 on 2023/4/6.
//

#include "Config.h"

void Config::Read(const char *configFile)
{
    int n;
    char tmp[200];
    mode = ini_getl("General", "mode", 0, configFile);
    n = ini_gets("RTK", "rover OBS source file", "", tmp, 200, configFile);
    if (n > 0)
        Config::rtkConfig.roverObsFile = tmp;
    n = ini_gets("RTK", "base OBS source file", "", tmp, 200, configFile);
    if (n > 0)
        Config::rtkConfig.baseObsFile = tmp;
    Config::rtkConfig.webOrFile = ini_getl("RTK", "web or file", 1, configFile);
    n = ini_gets("RTK", "base ip", "", tmp, 200, configFile);
    if (n > 0)
    {
        Config::rtkConfig.baseIP = new char[n + 1];
        for (int i = 0; i < n; i++)
            Config::rtkConfig.baseIP[i] = tmp[i];
        Config::rtkConfig.baseIP[n] = '\0';
    }
    n = ini_gets("RTK", "rover ip", "", tmp, 200, configFile);
    if (n > 0)
    {
        Config::rtkConfig.roverIP = new char[n + 1];
        for (int i = 0; i < n; i++)
            Config::rtkConfig.roverIP[i] = tmp[i];
        Config::rtkConfig.roverIP[n] = '\0';
    }
    Config::rtkConfig.basePort = ini_getl("RTK", "base port", 0, configFile);
    Config::rtkConfig.roverPort = ini_getl("RTK", "rover port", 0, configFile);
    Config::rtkConfig.basePos(1) = ini_getf("RTK", "base x pos", 0, configFile);
    Config::rtkConfig.basePos(2) = ini_getf("RTK", "base y pos", 0, configFile);
    Config::rtkConfig.basePos(3) = ini_getf("RTK", "base z pos", 0, configFile);
}
