/*
*************************************************************************************************************************************************************
This software and all the related documentation that are transmitted as a final report constitute Foregrund of the Research Agreement between IIT and Konica
and their use and exploitation are subject to the limitations contained therein.
*************************************************************************************************************************************************************
*/

#ifndef CROWD_DETECTOR_H
#define CROWD_DETECTOR_H

#include <string>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>



#include <algorithm>
#include <yarp/os/LogComponent.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IFrameTransform.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/cv/Cv.h>

#include <chrono>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#ifndef RAD2DEG
#define RAD2DEG (180.0 / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180.0)
#endif

#define TIMEOUT_MAX 100

 class MyModule : public yarp::os::RFModule
 {
     yarp::os::Port handlerPort; // a port to handle messages
     int count;
 public:
     double getPeriod();
     bool updateModule();
     bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
     bool configure(yarp::os::ResourceFinder &rf);
     bool interruptModule();
     bool close();

    // VARIABLES
        int n_humansT = 0;
        int n_humans = 0;


private:

    // VARIABLES
    std::string prefixName;
    std::string targetFrame;
    std::string remoteTCName;
    std::vector< std::string > allFrameIds;
    std::vector< std::string > filteredFrameIds;
    yarp::sig::Matrix transformMat;
    double m_area_radius;
    double m_x_human;
    double m_y_human;
    double m_up_b = 0;
    double m_lower_b = 0;
    double m_left_b = 0;
    double m_rigth_b = 0;
    double m_poi_y = 0;
    double m_poi_x = 0;
    time_t tstart, tend;



    yarp::dev::PolyDriver m_transformClientDriver;
    yarp::dev::IFrameTransform *m_transformClientInt;

 };


#endif // CROWD_DETECTOR_H
