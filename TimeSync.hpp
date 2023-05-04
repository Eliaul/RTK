//
// Created by 39894 on 2023/3/14.
//

#pragma once

#include "TimeSystem/GPSTime.h"
#include "OEM7File/OEM7Decoder.hpp"
#include "DataStructure/BestPos.h"
#include "Socket.h"
#include <iostream>

/// @brief 基站与流动站时间同步情况
/// @param roverTime
/// @param baseTime
/// @return 0为同步，1为流动站时间在基站之后，-1为流动站时间在基站之前
inline short Sync(GPSTime& roverTime, GPSTime& baseTime)
{
    auto lim = 100000000ns;
    if (roverTime - baseTime > lim)
        return 1;
    else if (roverTime - baseTime < -lim)
        return -1;
    else
        return 0;
}

inline bool IsRTK(OEM7Decoder<std::ifstream>& roverDecoder, OEM7Decoder<std::ifstream>& baseDecoder, std::map<Satellite, BaseEphemeris*>& ephemsMap)
{
    switch (Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime))
    {
        case 0: //同步，RTK
            return true;
        case -1: //流动站在基站之前，SPP
            return false;
        case 1: //流动站在基站之后，基站继续解码
        {
            if (!baseDecoder.stream.eof())
            {
                baseDecoder.Decode(ephemsMap);
                auto judge = IsRTK(roverDecoder, baseDecoder, ephemsMap);
                return judge;
            }
            return false;
        }
        default:
            return false;
    }
}

inline bool IsRTK(OEM7Decoder<std::stringstream>& roverDecoder, OEM7Decoder<std::stringstream>& baseDecoder, SOCKET& roverServer, SOCKET& baseServer, std::map<Satellite, BaseEphemeris*>& ephemsMap)
{
    auto judge = Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime);
    while (judge != 0)
    {
        if (judge == -1)
        {
            unsigned char roverBuff[20480];
            int len = recv(roverServer, reinterpret_cast<char*>(roverBuff), 20480, 0);
            if (len > 0)
            {
                std::stringstream roverStream;
                for (int i = 0; i < len; i++)
                    roverStream << roverBuff[i];
                roverDecoder.stream = std::move(roverStream);
                roverDecoder.Decode(ephemsMap);
                judge = Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime);
                //auto judge = IsRTK(roverDecoder, baseDecoder, roverServer, baseServer, ephemsMap);
            }
        }
        if (judge == 1)
        {
            unsigned char baseBuff[20480];
            int len = recv(baseServer, reinterpret_cast<char*>(baseBuff), 20480, 0);
            if (len > 0)
            {
                std::stringstream baseStream;
                for (int i = 0; i < len; i++)
                    baseStream << baseBuff[i];
                baseDecoder.stream = std::move(baseStream);
                baseDecoder.Decode(ephemsMap);
                judge = Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime);
            }
        }
    }
    return true;
}

inline bool GetData(SOCKET& socket, unsigned char* buff, OEM7Decoder<std::stringstream>& decoder, std::map<Satellite, BaseEphemeris*>& ephemsMap)
{
    //Sleep(700);
    int len = recv(socket, reinterpret_cast<char*>(buff), 40960, 0);
    std::cout << "len:" << len << std::endl;
    if (len <= 0)
        return false;
    decoder.stream.clear();
    for (int i = 0; i < len; i++)
        decoder.stream << buff[i];
    decoder.Decode(ephemsMap);
    return decoder.isAvailable;
}


inline void WebTimeSync(
    unsigned char* baseBuff, unsigned char* roverBuff, SOCKET& baseSocket, SOCKET& roverSocket,
    OEM7Decoder<std::stringstream>& baseDecoder, OEM7Decoder<std::stringstream>& roverDecoder,
    std::map<Satellite, BaseEphemeris*>& ephemsMap, int time
)
{
    Sleep(1050 - time);
    GetData(roverSocket, roverBuff, roverDecoder, ephemsMap);
    GetData(baseSocket, baseBuff, baseDecoder, ephemsMap);
    //Sleep(1200);
    auto judge = Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime);
    while (judge != 0)
    {
        if (judge == 1)
        {
            GetData(baseSocket, baseBuff, baseDecoder, ephemsMap);
            judge = Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime);
        }
        else if (judge == -1)
        {
            GetData(roverSocket, roverBuff, roverDecoder, ephemsMap);
            judge = Sync(roverDecoder.rangeMsg.msgData.gpsTime, baseDecoder.rangeMsg.msgData.gpsTime);
        }
    }
}

