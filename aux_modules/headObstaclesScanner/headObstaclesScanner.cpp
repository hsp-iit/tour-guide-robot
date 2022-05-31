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

#include "headObstaclesScanner.h"

double headScanner::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return 0.5;
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool headScanner::updateModule()
{
    count++;
    std::cout << "[" << count << "]"
              << " updateModule..." << '\n';

    // head encoders reading
    while (!iencs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }

    // print head position
    yarp::os::Bottle &head_heading = head_heading_port.prepare();
    head_heading.clear();
    head_heading.addFloat32(encoders(0));
    head_heading.addFloat32(encoders(1));
    head_heading_port.write();

    // head motion mode: sweep
    if (headModeName == "sweep")
    {
        sweepMode();
    }

    // head motion mode: trajectory
    if (headModeName == "trajectory")
    {
        double abs_angle = 0;
        double rel_angle = 0;
        bool short_trajectory = true;

        // get robot position
        if (!getRobotPosition())
            return false;

        // get trajectory (waypoints)
        getTrajectory();

        // calculate relative position
        for (int i = 0; i < abs_waypoints.rows(); i++)
        {
            for (int j = 0; j < robot_pose.cols(); j++)
                rel_waypoints(i, j) = abs_waypoints(i, j) - robot_pose(0, j);
        }

        for (int j = 0; j < robot_pose.cols(); j++)
            relative_target_loc[j] = relative_target_loc[j] - robot_pose(0, j);

        // calculate intersection with desired path
        if (rel_waypoints.rows() > 1)
        {
            waypoint_distance.resize(rel_waypoints.rows());
            for (int i = 0; i < rel_waypoints.rows(); i++)
            {
                waypoint_distance[i] = sqrt(pow(rel_waypoints(i, 0), 2) + pow(rel_waypoints(i, 1), 2));
                if (waypoint_distance[i] > circle_range)
                {
                    // abs_angle = atan2(rel_waypoints(i,1), rel_waypoints(i,0));
                    looking_point[0] = rel_waypoints(i, 0);
                    looking_point[1] = rel_waypoints(i, 1);
                    short_trajectory = false;
                    break;
                }
            }
            if (short_trajectory)
            {
                looking_point[0] = relative_target_loc(0);
                looking_point[1] = relative_target_loc(1);
                // abs_angle = atan2(relative_target_loc(1), relative_target_loc(0));
            }

            // show image
            drawImage();

            //                abs_angle = abs_angle * RAD2DEG;
            //                if (abs_angle < 0)
            //                    abs_angle = abs_angle + 360;
            //                rel_angle = abs_angle - robot_pose(0,2);
        }
        lookPoint();
#ifdef DEBUG

        std::cout << "relative position" << '\n';
        std::cout << rel_waypoints.toString() << '\n';
        std::cout << "absolute angle: " << abs_angle << '\n';
        std::cout << "relative angle: " << rel_angle << " from final target? " << short_trajectory << '\n';
        std::cout << "relative target: " << relative_target_loc.toString() << '\n';

        // std::cout << "INFO - pos control mode value: " << VOCAB_CM_POSITION << '\n';
        // std::cout << "INFO - pos direct control mode value: " << VOCAB_CM_POSITION_DIRECT << '\n';
        int actualMode = 0;
        icontrolMode->getControlMode(1, &actualMode);
        std::cout << "getControlMode: " << actualMode << '\n';
        if (nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached)
            std::cout << "INFO - goal reached \n";
        if (nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_idle)
            std::cout << "INFO - idle \n";
#endif
    }

    return true;
}

// Message handler. Just echo all received messages.
bool headScanner::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{
    std::cout << "Got something, echo is on" << '\n';
    if (command.get(0).asString() == "quit")
        return false;
    else
        reply = command;
    return true;
}
// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool headScanner::configure(yarp::os::ResourceFinder &rf)
{
    count = 0;
    done_run = true;
    robot_pose.resize(1, 3);
    robot_pose.zero();
    abs_objects.resize(0, 0);

    looking_point.resize(2);
    looking_point[0] = 0;
    looking_point[1] = 0;

    if (!handlerPort.open("/myModule"))
        return false;

    // optional, attach a port to the module
    // so that messages received from the port are redirected
    // to the respond method
    attach(handlerPort);

    // CONFIGURATION PARAMETERS (general)

    Bottle general_group = rf.findGroup("GENERAL");
    if (general_group.isNull())
    {
        yError() << "Missing GENERAL group!";
        return false;
    }

    if (!general_group.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return 1;
    }

    if (general_group.check("head_speed"))
    {
        head_speed = general_group.find("head_speed").asFloat64();
        ;
    }

    if (general_group.check("rotation_range"))
    {
        rotation_range = general_group.find("rotation_range").asFloat64();
    }

    if (!general_group.check("head_mode"))
    {
        fprintf(stderr, "WARNING parameter head_mode not specified, set default (modes: --head_mode sweep, trajectory, closer_corner)\n");
    }
    else
    {
        headModeName = general_group.find("head_mode").asString();
    }

    if (!general_group.check("head_pitch"))
    {
        fprintf(stderr, "WARNING parameter head_pitch not specified, set to 10 \n");
    }
    else
    {
        head_pitch = general_group.find("head_pitch").asFloat64();
    }

    // CONFIGURATION PARAMETERS (head behaviour)
    Bottle head_group = rf.findGroup("HEAD");
    if (head_group.isNull())
    {
        yWarning() << "Missing HEAD group, default parameters used";
        // return false;
    }

    if (head_group.check("circle_range"))
    {
        circle_range = head_group.find("circle_range").asFloat64();
    }

    std::string robotName = general_group.find("robot").asString();
    std::string remotePorts = "/";
    remotePorts += robotName;
    remotePorts += "/head";
    std::string localPorts = "/test/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts);   // local port names
    options.put("remote", remotePorts); // where we connect to

    // create a device
    // PolyDriver robotDevice(options);
    robotDevice = new PolyDriver(options);

    if (!robotDevice->isValid())
    {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    bool ok;
    ok = robotDevice->view(ipos);
    ok = ok && robotDevice->view(iencs);
    ok = ok && robotDevice->view(idirect);
    ok = ok && robotDevice->view(icontrolMode);

    if (!ok)
    {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    // get axes
    int nj = 0;
    ipos->getAxes(&nj);
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);

    // set accelerations and speeds
    int i;
    for (i = 0; i < nj; i++)
    {
        tmp[i] = 50.0;
    }
    ipos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++)
    {
        tmp[i] = head_speed;
        ipos->setRefSpeed(i, tmp[i]);
    }

    // fisrst read all encoders
    //
    printf("waiting for encoders");
    int cont_enc_lim = 0;
    while (!iencs->getEncoders(encoders.data()) && cont_enc_lim < 50)
    {
        // Time::delay(0.1);
        printf(".");
        cont_enc_lim++;
    }

    std::cout << "Done, cont_enc_lim:" << cont_enc_lim << '\n';

    command = encoders;

    // now set the head to a neutral position
    command[0] = 0;
    command[1] = 0;
    ipos->positionMove(command.data());

    printf("waiting for checkMotionDone");
    cont_enc_lim = 0;
    bool done = false;
    while (!done && cont_enc_lim < 50)
    {
        ipos->checkMotionDone(&done);
        Time::delay(0.1);
        cont_enc_lim++;
    }
    std::cout << "Done, cont_enc_lim:" << cont_enc_lim << '\n';

    // open port to send head heading
    head_heading_port.open(m_local_name_prefix + "/head_position:o");

    // open localization and navigation clients
    if (headModeName == "trajectory")
    {
        // parameters for localization and navigation servers

        Bottle navigation_group = rf.findGroup("NAVIGATION");
        if (navigation_group.isNull())
        {
            yWarning() << "Missing NAVIGATION group, default parameters used";
            // return false;
        }

        if (general_group.check("local"))
        {
            m_local_name_prefix = general_group.find("local").asString();
        }
        if (navigation_group.check("remote_localization"))
        {
            m_remote_localization = navigation_group.find("remote_localization").asString();
        }
        if (navigation_group.check("remote_navigation"))
        {
            m_remote_navigation = navigation_group.find("remote_navigation").asString();
        }
        if (navigation_group.check("remote_map"))
        {
            m_remote_map = navigation_group.find("remote_map").asString();
        }

        // open the localization interface
        Property loc_options;
        loc_options.put("device", "localization2D_nwc_yarp");
        loc_options.put("local", m_local_name_prefix + "/localizationClient");
        loc_options.put("remote", m_remote_localization);
        if (m_pLoc.open(loc_options) == false)
        {
            yError() << "Unable to open localization driver";
            return false;
        }
        m_pLoc.view(m_iLoc);
        if (m_pLoc.isValid() == false || m_iLoc == 0)
        {
            yError() << "Unable to view localization interface";
            return false;
        }

        // open the navigation interface

        Property nav_options;
        nav_options.put("device", "navigation2D_nwc_yarp");
        nav_options.put("local", m_local_name_prefix + "/navigation2D_nwc_yarp");
        nav_options.put("navigation_server", m_remote_navigation);
        nav_options.put("map_locations_server", m_remote_map);
        nav_options.put("localization_server", m_remote_localization);
        if (m_pNav.open(nav_options) == false)
        {
            yError() << "Unable to open navigation2D_nwc_yarp";
            return false;
        }
        m_pNav.view(m_iNav);
        if (m_iNav == 0)
        {
            yError() << "Unable to open navigation interface";
            return false;
        }
    }

    // show map and open output port for image
    if (headModeName == "trajectory")
    {
        // read map
        if (head_group.check("map_name"))
        {
            map_name = head_group.find("map_name").asString();
        }
        if (head_group.check("map_resolution"))
        {
            map_resolution = head_group.find("map_resolution").asFloat64();
        }

        // read from nav server

        m_iNav->getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum::global_map, m_current_map);

        m_current_map.getMapImage(image_map);

        // create an opencv mat from yarp
        image_map_cv = yarp::cv::toCvMat(image_map);

        // src = cv::imread( map_name, 1 );

        // open image port and send image
        imagePort.open(m_local_name_prefix + "/rgb:o");

        drawImage();

        // std::cout << "corners found:  " << rel_map_corners.rows() << '\n';
        // std::cout << "relative corners: \n " << rel_map_corners.toString() << '\n';
    }

    return true;
}

// Interrupt function.
bool headScanner::interruptModule()
{
    std::cout << "Interrupting your module, for port cleanup" << '\n';
    return true;
}

// Close function, to perform cleanup.
bool headScanner::close()
{
    icontrolMode->setControlMode(1, VOCAB_CM_POSITION);
    // optional, close port explicitly
    std::cout << "Calling close function\n";
    robotDevice->close();
    handlerPort.close();

    if (m_pNav.isValid())
        m_pNav.close();
    m_iNav = nullptr;

    m_port_local_trajectory.interrupt();
    m_port_local_trajectory.close();

    m_port_opti_points.interrupt();
    m_port_opti_points.close();

    head_heading_port.interrupt();
    head_heading_port.close();

    return true;
}

// head mode sweep
bool headScanner::sweepMode()
{
    if (std::abs(encoders(1) - command(1)) < 1)
    {
        done_run = true;
    }
    else
    {
        done_run = false;
    }

    if (done_run)
    {
        com_count++;
        if (com_count % 2)
        {
            command[0] = head_pitch;
            command[1] = rotation_range;
        }
        else
        {
            command[0] = head_pitch;
            command[1] = -rotation_range;
        }
        ipos->positionMove(command.data());
    }
    return true;
}

bool headScanner::getTrajectory()
{

    double rel_x = 0;
    double rel_y = 0;
    double rel_t = 0;
    // m_iNav->getRelativeLocationOfCurrentTarget(rel_x, rel_y, rel_t);
    // relative_target_loc[0] = rel_x;
    // relative_target_loc[1] = rel_y;
    // relative_target_loc[2] = rel_t;

    if (m_iNav->getAbsoluteLocationOfCurrentTarget(m_target_data))
    {
        relative_target_loc[0] = m_target_data.x;
        relative_target_loc[1] = m_target_data.y;
        relative_target_loc[2] = m_target_data.theta;
        if (relative_target_loc[2] < 0)
            relative_target_loc[2] = relative_target_loc[2] + 360;
    }

    m_iNav->getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum::global_trajectory, m_all_waypoints);

    abs_waypoints.resize(m_all_waypoints.size(), 3);
    rel_waypoints.resize(m_all_waypoints.size(), 3);

    for (int i = 0; i < m_all_waypoints.size(); i++)
    {
        abs_waypoints(i, 0) = m_all_waypoints[i].x;
        abs_waypoints(i, 1) = m_all_waypoints[i].y;
        abs_waypoints(i, 2) = -1;
        // abs_waypoints(i,2) = m_all_waypoints[i].theta;
        // if (abs_waypoints(i,2)<0)
        //     abs_waypoints(i,2) = abs_waypoints(i,2) + 360;
    }

#ifdef DEBUG_LV2
    std::cout << "abs_waypoints.toString(): " << '\n';
    // std::cout << m_all_waypoints.toString() << '\n';
    std::cout << abs_waypoints.toString() << '\n';
#endif

    return true;
}

bool headScanner::getRobotPosition()
{
    bool ret = m_iLoc->getCurrentPosition(m_localization_data);
    if (ret)
    {
        m_loc_timeout_counter = 0;
        robot_pose(0, 0) = m_localization_data.x;
        robot_pose(0, 1) = m_localization_data.y;
        robot_pose(0, 2) = m_localization_data.theta;
        if (robot_pose(0, 2) < 0)
            robot_pose(0, 2) = robot_pose(0, 2) + 360;
    }
    else
    {
        m_loc_timeout_counter++;
        if (m_loc_timeout_counter > TIMEOUT_MAX)
            m_loc_timeout_counter = TIMEOUT_MAX;
        yError(" timeout, no localization data received!\n");
        return false;
    }

#ifdef DEBUG_LV3
    std::cout << "localization data" << '\n';
    // std::cout << m_localization_data.toString() << '\n';
    std::cout << robot_pose.toString() << '\n';
#endif

    return true;
}

bool headScanner::drawImage()
{
    // colors
    Scalar c_red = Scalar(0, 0, 255);
    Scalar c_blue = Scalar(255, 0, 0);
    Scalar c_green = Scalar(0, 255, 0);

    // Read image
    // src = cv::imread( map_name, 1 );

    src = image_map_cv;

    if (~src.empty())
    {
        copy = src.clone();
        cv::RNG rng(12345);

        // Create Window
        // namedWindow( source_window, CV_WINDOW_AUTOSIZE );
        // imshow( source_window, src );

        // Draw waypoints
        int r = 4;
        for (int i = 0; i < abs_waypoints.rows(); i++)
        {
            circle(copy, Point(round(abs_waypoints(i, 1) / map_resolution), round(abs_waypoints(i, 0) / map_resolution)), r, c_green, -1, 8, 0);
        }

        // Draw robot position
        Point2f opencv_robot_pos;
        Point2f opencv_robot_pos_2;
        opencv_robot_pos.x = robot_pose(0, 1) / map_resolution;
        opencv_robot_pos.y = robot_pose(0, 0) / map_resolution;

        opencv_robot_pos_2.x = (robot_pose(0, 1) + 0.5 * sin(robot_pose(0, 2) * DEG2RAD)) / map_resolution;
        opencv_robot_pos_2.y = (robot_pose(0, 0) + 0.5 * cos(robot_pose(0, 2) * DEG2RAD)) / map_resolution;

        circle(copy, opencv_robot_pos, r * 2, c_red, -1, 8, 0);
        line(copy, opencv_robot_pos, opencv_robot_pos_2, c_red, 2, 8, 0);

        // draw robot camera FOV
        Point2f opencv_left_corner;
        Point2f opencv_right_corner;

        opencv_left_corner.x = (robot_pose(0, 1) + camera_max_considered_radius * sin((robot_pose(0, 2) + encoders(1) - camera_fov / 2) * DEG2RAD)) / map_resolution;
        opencv_left_corner.y = (robot_pose(0, 0) + camera_max_considered_radius * cos((robot_pose(0, 2) + encoders(1) - camera_fov / 2) * DEG2RAD)) / map_resolution;

        opencv_right_corner.x = (robot_pose(0, 1) + camera_max_considered_radius * sin((robot_pose(0, 2) + encoders(1) + camera_fov / 2) * DEG2RAD)) / map_resolution;
        opencv_right_corner.y = (robot_pose(0, 0) + camera_max_considered_radius * cos((robot_pose(0, 2) + encoders(1) + camera_fov / 2) * DEG2RAD)) / map_resolution;
        line(copy, opencv_robot_pos, opencv_left_corner, c_green, 1, 8, 0);
        line(copy, opencv_robot_pos, opencv_right_corner, c_green, 1, 8, 0);
        line(copy, opencv_right_corner, opencv_left_corner, c_green, 1, 8, 0);

        // draw lqr path
        Point2f opencv_traj_1;
        Point2f opencv_traj_2;
        if (lqrTraj.rows() > 1)
        {
            int count_dir = 0;
            for (int i = 1; i < lqrTraj.rows(); i++)
            {
                // draw trajectory
                opencv_traj_1.x = lqrTraj(i, 1) / map_resolution;
                opencv_traj_1.y = lqrTraj(i, 0) / map_resolution;
                opencv_traj_2.x = lqrTraj(i - 1, 1) / map_resolution;
                opencv_traj_2.y = lqrTraj(i - 1, 0) / map_resolution;
                cv::line(copy, opencv_traj_1, opencv_traj_2, c_green, 1, 8, 0);

                count_dir = count_dir + 5;
                if (count_dir < lqrTraj.rows())
                {
                    // draw orientation
                    opencv_traj_1.x = lqrTraj(count_dir - 1, 1) / map_resolution;
                    opencv_traj_1.y = lqrTraj(count_dir - 1, 0) / map_resolution;
                    opencv_traj_2.x = (lqrTraj(count_dir - 1, 1) + 0.5 * sin(lqrTraj(count_dir - 1, 2) * DEG2RAD)) / map_resolution;
                    opencv_traj_2.y = (lqrTraj(count_dir - 1, 0) + 0.5 * cos(lqrTraj(count_dir - 1, 2) * DEG2RAD)) / map_resolution;
                    cv::line(copy, opencv_traj_1, opencv_traj_2, c_blue, 1, 8, 0);
                }
            }
            // draw first segment (proportional to speed)
            opencv_traj_1.x = ((lqrTraj(1, 1) - robot_pose(0, 1)) * 30 + robot_pose(0, 1)) / map_resolution;
            opencv_traj_1.y = ((lqrTraj(1, 0) - robot_pose(0, 0)) * 30 + robot_pose(0, 0)) / map_resolution;
            opencv_traj_2.x = lqrTraj(0, 1) / map_resolution;
            opencv_traj_2.y = lqrTraj(0, 0) / map_resolution;
            cv::line(copy, opencv_traj_1, opencv_traj_2, c_blue, 1, 8, 0);
        }

        // Send image to the door
        ImageOf<PixelRgb> &img = imagePort.prepare();

        img = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(copy);

        imagePort.write();
    }
    return true;
}

bool headScanner::lookPoint()
{
    double abs_angle = 0;
    double rel_angle;

    abs_angle = atan2(looking_point[1], looking_point[0]);
    abs_angle = abs_angle * RAD2DEG;
    if (abs_angle < 0)
        abs_angle = abs_angle + 360;

    rel_angle = abs_angle - robot_pose(0, 2);

    command[0] = head_pitch;

    // stop head when target is reached

    m_iNav->getNavigationStatus(nav_status);
    if (nav_status != yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_moving) //((nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached) || nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_idle)
    {
        rel_angle = 0;
        command[0] = 0;
    }
    else
    {
        head_movement_when_idle = true;
    }

    // move robot head

    if (rel_angle > 180)
        command[1] = rel_angle - 360;
    else
        command[1] = rel_angle;

    // pos control mode: 7565168 ---- pos direct control mode: 1685286768

    if (head_movement_when_idle)
    {
        if (rel_angle == 0)
        {
            head_movement_when_idle = false;
        }
        if (std::abs(encoders(1) - command(1)) < 3)
        {
            icontrolMode->setControlMode(1, VOCAB_CM_POSITION_DIRECT);
            idirect->setPosition(1, command[1]);
            idirect->setPosition(0, command[0]);
        }
        else
        {
            icontrolMode->setControlMode(1, VOCAB_CM_POSITION);
            ipos->positionMove(command.data());
        }
    }

#ifdef DEBUG_LV3
    std::cout << "LOOKING POINT:" << '\n';
    std::cout << "robot position X: " << robot_pose(0, 0) << " Y: " << robot_pose(0, 1) << " theta: " << robot_pose(0, 2) << '\n';
    std::cout << "relative looking point X: " << looking_point[0] << " Y: " << looking_point[1] << '\n';
    std::cout << "point angle in robot reference system: " << abs_angle << '\n';
    std::cout << "point angle relative to robot head: " << rel_angle << '\n';
    std::cout << "commanded angle: " << command[1] << '\n';
    std::cout << "actual angle: " << encoders[1] << '\n';
#endif
    return true;
}

bool headScanner::lookRelativeAngle()
{
    double rel_angle = relative_commanded_angle;
    // stop head when target is reached
    m_iNav->getNavigationStatus(nav_status);
    command[0] = head_pitch;
    if ((nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached) || nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_idle)
    {
        rel_angle = 0;
        command[0] = 0;
    }

    // move robot head

    if (rel_angle > 180)
        command[1] = rel_angle - 360;
    else
        command[1] = rel_angle;

    // pos control mode: 7565168 ---- pos direct control mode: 1685286768

    if (std::abs(encoders(1) - command(1)) < 3)
    {
        icontrolMode->setControlMode(1, VOCAB_CM_POSITION_DIRECT);
        idirect->setPosition(1, command[1]);
        idirect->setPosition(0, command[0]);
    }
    else
    {
        icontrolMode->setControlMode(1, VOCAB_CM_POSITION);
        ipos->positionMove(command.data());
    }

#ifdef DEBUG_LV3
    std::cout << "LOOKING RELATIVE ANGLE:" << '\n';
    std::cout << "point angle relative to robot head: " << rel_angle << '\n';
    std::cout << "commanded angle: " << command[1] << '\n';
    std::cout << "actual angle: " << encoders[1] << '\n';
#endif
    return true;
}

yarp::sig::Matrix headScanner::getLqrTrajectory()
{
    yarp::sig::Matrix m_lqrTraj;
    yarp::sig::Matrix m_lqrTraj_2;
    yarp::os::Bottle *trajectory = m_port_local_trajectory.read(false);
    if (trajectory)
    {
        string frame = trajectory->get(0).asString();
        m_lqrTraj.resize(trajectory->size() - 1, 7);
        m_lqrTraj_2.resize(trajectory->size() - 1, 7);
        for (size_t i = 1; i < trajectory->size(); i++)
        {
            Bottle *list_traj = trajectory->get(i).asList();
            m_lqrTraj(i - 1, 0) = list_traj->get(0).asFloat64(); // x
            m_lqrTraj(i - 1, 1) = list_traj->get(1).asFloat64(); // y
            m_lqrTraj(i - 1, 2) = list_traj->get(2).asFloat64(); // theta
            m_lqrTraj(i - 1, 3) = list_traj->get(3).asFloat64(); // vel x
            m_lqrTraj(i - 1, 4) = list_traj->get(4).asFloat64(); // vel y
            m_lqrTraj(i - 1, 5) = list_traj->get(5).asFloat64(); // vel theta
            m_lqrTraj(i - 1, 6) = list_traj->get(6).asFloat64(); // time stamp
        }
        m_lqrTraj_2 = m_lqrTraj;

        // recalculate trajectory aligning robot heading and first trajectory segment
        float rel_angle = atan2(m_lqrTraj(1, 1) - m_lqrTraj(0, 1), m_lqrTraj(1, 0) - m_lqrTraj(0, 0)) * RAD2DEG;
        rel_angle = (robot_pose(0, 2) - rel_angle) * DEG2RAD;

        int k = 1;
        for (int i = 0; i < m_lqrTraj.rows(); i++)
        {
            m_lqrTraj_2(i, 0) = (robot_pose(0, 0) + ((m_lqrTraj(i, 0) - k * m_lqrTraj(0, 0)) * cos(rel_angle) - (m_lqrTraj(i, 1) - k * m_lqrTraj(0, 1)) * sin(rel_angle)));
            m_lqrTraj_2(i, 1) = (robot_pose(0, 1) + ((m_lqrTraj(i, 0) - k * m_lqrTraj(0, 0)) * sin(rel_angle) + (m_lqrTraj(i, 1) - k * m_lqrTraj(0, 1)) * cos(rel_angle)));
        }

        // calculate absolute heading
        for (int i = 1; i < m_lqrTraj.rows(); i++)
        {
            m_lqrTraj_2(i - 1, 2) = atan2(m_lqrTraj_2(i, 1) - m_lqrTraj_2(i - 1, 1), m_lqrTraj_2(i, 0) - m_lqrTraj_2(i - 1, 0)) * RAD2DEG;
            if (m_lqrTraj_2(i - 1, 2) < 0)
                m_lqrTraj_2(i - 1, 2) = m_lqrTraj_2(i - 1, 2) + 360;
        }
        m_lqrTraj_2(m_lqrTraj.rows() - 1, 2) = m_lqrTraj_2(m_lqrTraj.rows() - 2, 2);
    }
    else
    {
        m_lqrTraj_2.resize(0, 0);
    }

#ifdef DEBUG_LV2
    // cout << "m_lqrTraj_2.rows(): " << m_lqrTraj_2.rows() << '\n' << m_lqrTraj_2.toString() << '\n' ;
#endif

    return m_lqrTraj_2;
}