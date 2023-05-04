//
// Created by 39894 on 2023/4/18.
//

#include "RTKMode.h"
#include "FixedSolution.h"
#include "OEM7File/OEM7Decoder.hpp"
#include "ErrorDetector.hpp"
#include "TimeSync.hpp"
#include "Lambda.h"
#include "Export.h"
#include "Socket.h"
#include <sstream>

using namespace std;

void WebRTK(
    const char *roverIP, const unsigned short roverPort,
    const char *baseIP, const unsigned short basePort
)
{
    SOCKET baseServer;
    SOCKET roverServer;
    bool isBaseOpen = OpenSocket(baseServer, baseIP, basePort);
    bool isRoverOpen = OpenSocket(roverServer, roverIP, roverPort);
    OEM7Decoder<stringstream> roverDecoder;
    OEM7Decoder<stringstream> baseDecoder;

    ErrorDetector<Range> roverErrorDetector;
    ErrorDetector<Range> baseErrorDetector;
    ErrorDetector<SingleDifferenceRange> sdErrorDetector;
    std::map<Satellite, BaseEphemeris*> ephemsMap;

    int total = 0;
    int rtk = 0;
    unsigned char* roverBuff = new unsigned char[40960];
    unsigned char* baseBuff = new unsigned char[40960];
    int sleepTime = -1000;
    std::ofstream ofs("fix.log", std::ios::out | std::ios::trunc);
    std::ofstream ofs_float("float.log", std::ios::out | std::ios::trunc);
    std::ofstream ofs1("rover.bin", std::ios::out | std::ios::trunc | std::ios::binary);
    std::ofstream ofs2("base.bin", std::ios::out | std::ios::trunc | std::ios::binary);
    if (isBaseOpen && isRoverOpen)
    {
        while (true)
        {
//            int len1 = recv(roverServer, reinterpret_cast<char*>(roverBuff), 40960, 0);
//            int len2 = recv(baseServer, reinterpret_cast<char*>(baseBuff), 40960, 0);
//            if (len1 > 0)
//                for (int i = 0; i < len1; i++)
//                    ofs1 << roverBuff[i];
//            if (len2 > 0)
//                for (int i = 0; i < len2; i++)
//                    ofs2 << baseBuff[i];
//            cout << len1 << " " << len2 << endl;

            WebTimeSync(baseBuff, roverBuff, baseServer, roverServer, baseDecoder, roverDecoder, ephemsMap, sleepTime);
            auto start = std::chrono::high_resolution_clock::now();
            total++;
            if (!roverDecoder.isAvailable || !baseDecoder.isAvailable)
            {
                auto end = std::chrono::high_resolution_clock::now();
                sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                continue;
            }
            roverErrorDetector.ErrorDetect(roverDecoder.rangeMsg.msgData);
            SPPResult<CoordinateSystem::WGS84> rovRes;
            rovRes.SPPCalculate(roverDecoder.rangeMsg.msgData, ephemsMap);
            if (!rovRes.isAvailable)
            {
                auto end = std::chrono::high_resolution_clock::now();
                sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                continue;
            }
            baseErrorDetector.ErrorDetect(baseDecoder.rangeMsg.msgData);
            SPPResult<CoordinateSystem::WGS84> baseRes;
            baseRes.SPPCalculate(baseDecoder.rangeMsg.msgData, ephemsMap);
            if (!baseRes.isAvailable)
            {
                auto end = std::chrono::high_resolution_clock::now();
                sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                continue;
            }
            SingleDifferenceRange sdRange;
            sdRange.DoSingleDifference(roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, ephemsMap);
            sdErrorDetector.ErrorDetect(sdRange);
            DoubleDifferenceOBS ddOBS;
            ddOBS.DetermineRefSate(sdRange, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData);
            FloatSolution floatSolution;
            floatSolution.RelativePositioning(sdRange, ddOBS, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, baseDecoder.bestPos, rovRes);
            if (!floatSolution.isAvailable)
            {
                auto end = std::chrono::high_resolution_clock::now();
                sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                continue;
            }
            auto fixedAmbiguity = Lambda(2, floatSolution.ambiguousArgs, floatSolution.ambiguousQ);
            if (fixedAmbiguity.has_value())
            {
                auto& [F, ratio] = fixedAmbiguity.value();
                if (ratio < 3)
                {
                    auto end = std::chrono::high_resolution_clock::now();
                    sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    continue;
                }
                FixedSolution fixedSolution;
                rtk++;
                fixedSolution.ratio = ratio;
                fixedSolution.GetFixedSolution(sdRange, ddOBS, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, baseDecoder.bestPos, F, floatSolution);
                ofs << ExportCSV(fixedSolution, ddOBS, floatSolution, rtk, total) << endl;
                ofs_float << ExportCSV(floatSolution) << endl;
                cout << "rover time: " << roverDecoder.rangeMsg.msgData.gpsTime << endl
                     << "base time: " << baseDecoder.rangeMsg.msgData.gpsTime << endl
                     << "fixed: " << fixedSolution.roverPos << " ratio: " << ratio << endl
                     << std::format("{0}/{1}\n", rtk, total) << endl;
            }
            if (rtk % 100 == 0)
            {
                stringstream rovertmp, basetmp;
                rovertmp << roverDecoder.stream.rdbuf();
                roverDecoder.stream = std::move(rovertmp);
                basetmp << baseDecoder.stream.rdbuf();
                baseDecoder.stream = std::move(basetmp);
            }
            auto end = std::chrono::high_resolution_clock::now();
            sleepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            //cout << sleepTime << "ms" << endl << endl;
        }
    }
}

void FileRTK(std::string roverPath, std::string basePath, XYZCoordinate basePos)
{
    std::ifstream baseIfs(basePath, std::ios::in | std::ios::binary);
    std::ifstream roverIfs(roverPath, std::ios::in | std::ios::binary);
    if (!baseIfs.is_open() || !roverIfs.is_open())
        return;
    OEM7Decoder<std::ifstream> baseDecoder(baseIfs);
    OEM7Decoder<std::ifstream> roverDecoder(roverIfs);
    ErrorDetector<Range> roverErrorDetector;
    ErrorDetector<Range> baseErrorDetector;
    ErrorDetector<SingleDifferenceRange> sdErrorDetector;
    std::ofstream ofs("fix.log", std::ios::out | std::ios::trunc);
    std::ofstream ofs_float("float.log", std::ios::out | std::ios::trunc);
    std::map<Satellite, BaseEphemeris*> ephemsMap;
    int i = 0;
    int j = 0;
    int rtkfix = 0, total = 0, rtkfloat = 0;
    while (!roverDecoder.stream.eof())
    {
        roverDecoder.Decode(ephemsMap);
        if (IsRTK(roverDecoder, baseDecoder, ephemsMap) && roverDecoder.isAvailable && baseDecoder.isAvailable)
        {
            total++;
            i++;
            roverErrorDetector.ErrorDetect(roverDecoder.rangeMsg.msgData);
            SPPResult<CoordinateSystem::WGS84> rovRes;
            rovRes.SPPCalculate(roverDecoder.rangeMsg.msgData, ephemsMap);
            if (!rovRes.isAvailable)
                continue;
            baseErrorDetector.ErrorDetect(baseDecoder.rangeMsg.msgData);
            SPPResult<CoordinateSystem::WGS84> baseRes;
            baseRes.SPPCalculate(baseDecoder.rangeMsg.msgData, ephemsMap);
            if (!baseRes.isAvailable)
                continue;
            SingleDifferenceRange sdRange;
            sdRange.DoSingleDifference(roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, ephemsMap);
            sdErrorDetector.ErrorDetect(sdRange);
            DoubleDifferenceOBS ddOBS;
            ddOBS.DetermineRefSate(sdRange, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData);
            FloatSolution floatSolution;
            baseDecoder.bestPos.xyzPos = basePos;
            floatSolution.RelativePositioning(sdRange, ddOBS, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, baseDecoder.bestPos, rovRes);
            if (!floatSolution.isAvailable)
                continue;
//            int n = floatSolution.ambiguousArgs.Length();
//            double* a = new double[n];
//            double* F = new double[n * 2];
//            double* s = new double[2];
//            double* Q = new double[n * n];
//            floatSolution.ambiguousArgs.ToPointer(a);
//            floatSolution.ambiguousQ.ToPointer(Q);
//            lambda(n, 2, a, Q, F, s);
//            if (s[1] / s[0] > 3)
//            {
//                Matrix FF(n, 2);
//                for (int k = 0; k < n; k++)
//                {
//                    FF(k + 1, 1) = F[k];
//                    FF(k + 1, 2) = F[k + n];
//                }
//                FixedSolution fixedSolution;
//                fixedSolution.ratio = s[1] / s[0];
//                fixedSolution.GetFixedSolution(sdRange, ddOBS, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, baseDecoder.bestPos, FF, floatSolution);
//                ofs << fixedSolution;
//            }
//            delete[] a;
//            delete[] F;
//            delete[] s;
//            delete[] Q;
            auto fixedAmbiguity = Lambda(2, floatSolution.ambiguousArgs, floatSolution.ambiguousQ);
            if (fixedAmbiguity.has_value())
            {
                auto& [F, ratio] = fixedAmbiguity.value();
                if (ratio < 3)
                {
                    rtkfloat++;
                    continue;
                }
                FixedSolution fixedSolution;
                fixedSolution.ratio = ratio;
                rtkfix++;
                fixedSolution.GetFixedSolution(sdRange, ddOBS, roverDecoder.rangeMsg.msgData, baseDecoder.rangeMsg.msgData, baseDecoder.bestPos, F, floatSolution);
                ofs << ExportCSV(fixedSolution, ddOBS, floatSolution) << endl;
                ofs_float << ExportCSV(floatSolution) << endl;
            }
        }
        else
        {
            j++;
        }
    }
    //std::cout << i << " " << j;
    ofs << "fixed:" << rtkfix << " float:" << rtkfloat << " total:" << total;
    roverDecoder.Close();
    baseDecoder.Close();
    ofs.close();
    ofs_float.close();
}
