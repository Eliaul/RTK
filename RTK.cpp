#include "DataStructure/PositionResult.hpp"
#include "TimeSync.hpp"
#include "OEM7File/OEM7Decoder.hpp"
#include "SingleDifference.h"
#include "ErrorDetector.hpp"
#include "DoubleDifference.h"
#include "RelativePositioning.hpp"
#include "LinerAlgebra/Algorithm.h"
#include "Lambda.h"
#include "lambda1.h"
#include "FixedSolution.h"
#include "Socket.h"
#include "Config/minIni.h"
#include "Config/Config.h"
#include <chrono>
#include "Export.h"
#include "RTKMode.h"
//#include "CoordinateSystem/XYZCoordinate.hpp"

using namespace std;

constexpr char configFile[] = "../config.ini";

int  main()
{
    // short-baseline
//        XYZCoordinate xyz(-2267804.5263, 5009342.3723, 3220991.8632);
//        auto blh = XYZCoordinate::ToBLH<CoordinateSystem::WGS84>(xyz);
    // zero-baseline
    //    XYZCoordinate xyz(-2267799.4107, 5009331.0725, 3220984.5485);
    //    auto blh = XYZCoordinate::ToBLH<CoordinateSystem::WGS84>(xyz);

    Config config;
    config.Read(configFile);
    if (config.mode == 1)
    {

        if (config.rtkConfig.webOrFile == 0)
        {
            auto roverIP = config.rtkConfig.roverIP;
            auto roverPort = config.rtkConfig.roverPort;
            auto baseIP = config.rtkConfig.baseIP;
            auto basePort = config.rtkConfig.basePort;
            WebRTK(roverIP, roverPort, baseIP, basePort);
        }
        if (config.rtkConfig.webOrFile == 1)
        {
            string basePath;
            string roverPath;
            if (config.rtkConfig.baseObsFile.has_value() && config.rtkConfig.roverObsFile.has_value())
            {
                basePath = config.rtkConfig.baseObsFile.value();
                roverPath = config.rtkConfig.roverObsFile.value();
            }
            else
                return 0;
            FileRTK(roverPath, basePath, config.rtkConfig.basePos);
        }
    }

    return 0;
}