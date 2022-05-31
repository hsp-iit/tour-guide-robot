/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef HEADOBSTACLES_H
#define HEADOBSTACLES_H
#include <string>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <glpk.h>

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

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/cv/Cv.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

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

#define DEBUG
// #define DEBUG_LV2
//#define DEBUG_LV3

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace cv;

class headScanner : public yarp::os::RFModule
{

public:
    // PARAMETERS

    // head behaviour default parameters
    double head_speed = 25.0;                   //head rotational speed
    double rotation_range = 35.0;               //head rotation range (+/- rotation_range)
    double circle_range = 1;                    //distance that determines the waypoint at which we are looking at
    double camera_fov = 70;                     //camera field of view
    double camera_max_considered_radius = 3.5;  //maximum distance from the robot to detect obstacles
    double head_pitch = 10;                    //fixed robot head pitch

    // devices default parameters
    std::string  m_remote_localization = "/localization2D_nws_yarp";
    std::string  m_remote_map = "/map2D_nws_yarp";
    std::string  m_remote_navigation = "/navigation2D_nws_yarp";
    std::string  m_local_name_prefix = "/headObstaclesScanner";
    std::string  headModeName = "sweep";
    //std::string  map_name = "/dockersharedfolder/testopencv/test4/squirico_map.png";
    //std::string  map_name = "/dockersharedfolder/testopencv/test4/mymap_isaac3.png";
    std::string  map_name = "/test/manual/load.png";

    yarp::dev::Nav2D::MapGrid2D m_current_map;
    yarp::sig::ImageOf< yarp::sig::PixelRgb > image_map;
    cv::Mat image_map_cv;
    double  map_resolution = 0.05;              //resolution of the map used

    // optimization default parameters
    double std_weight_corners = 0.1;
    double std_weight_objects = 4;
    double std_weight_waypoints = 1;



    // DEVICES
    PolyDriver *robotDevice;
    IPositionControl *ipos;
    IEncoders *iencs;
    IPositionDirect *idirect;
    IControlMode *icontrolMode;

    PolyDriver      m_pNav;
    PolyDriver      m_pLoc;

    yarp::dev::Nav2D::INavigation2D*        m_iNav;
    yarp::dev::Nav2D::ILocalization2D*      m_iLoc;

    yarp::dev::Nav2D::Map2DPath  m_all_waypoints;
    yarp::dev::Nav2D::Map2DLocation        m_localization_data;
    yarp::dev::Nav2D::Map2DLocation        m_target_data;

    // SERVICE VARIABLES
    std::string m_port_local_trajectory_name;
    std::string m_port_opti_points_name;

    yarp::os::Port handlerPort; // a port to handle messages
    int count;

    Vector encoders;
    Vector command;
    Vector tmp;

    double relative_commanded_angle = 0;

    bool done_run;
    long int com_count = 0;
    int m_loc_timeout_counter = 0;

    yarp::sig::Matrix abs_waypoints;
    yarp::sig::Matrix abs_objects;
    yarp::sig::Matrix t_abs_objects;
    yarp::sig::Matrix rel_waypoints;
    yarp::sig::Matrix map_corners;
    vector<Point2f> opencv_corners;
    yarp::sig::Matrix rel_map_corners;
    yarp::sig::Matrix abs_map_corners;
    yarp::sig::Matrix robot_pose;
    std::vector<double> waypoint_distance;
    Vector relative_target_loc = {0, 0, 0};
    yarp::dev::Nav2D::NavigationStatusEnum  nav_status;
    std::vector<double> looking_point;
    bool head_movement_when_idle = true;

    std::string source_window = "Image";
    Mat copy;
    Mat src, src_gray;

    yarp::sig::Matrix lqrTraj;

    yarp::os::BufferedPort<ImageOf<PixelRgb> > imagePort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_local_trajectory;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_opti_points;
    yarp::os::BufferedPort<yarp::os::Bottle> head_heading_port;

    int closer_point_index = 0;

    yarp::conf::vocab32_t VOCAB_CM_POSITION        =   yarp::os::createVocab32('p','o','s');
    yarp::conf::vocab32_t VOCAB_CM_POSITION_DIRECT =   yarp::os::createVocab32('p','o','s','d');

    //int *actualMode;



    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool interruptModule();
    virtual bool updateModule();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    yarp::sig::Matrix getLqrTrajectory ();
    bool lookRelativeAngle();
    bool lookPoint();
    bool drawImage();
    bool getRobotPosition();
    bool getTrajectory();
    bool sweepMode();


};

#endif
