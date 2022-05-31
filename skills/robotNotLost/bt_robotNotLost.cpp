/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "bt_robotNotLost.h"

YARP_LOG_COMPONENT(ROBOTNOTLOST_CONDITION, "behavior_tour_robot.skills.robotnotlost_condition", yarp::os::Log::TraceType)

robotNotLost::robotNotLost(std::string name) : m_name(name),
                                               m_status(SKILL_IDLE),
                                               m_robotNotLost(true),
                                               m_period(0.5),
                                               m_pBtName("/" + name + "/BT_rpc/server"),
                                               private_nh("/localizationDiagnosticsNode"),
                                               m_tourManagerPortName("/" + name + "/TourManager/thrift:c")
{

    // Visualisations
    marker_count = 0;

    // Total number of accumulated faults
    amcl_total_faults = 0;
    odom_total_faults = 0;

    // Amcl-Amcl fault detection
    amcl_temporal_angular_fault = false;
    amcl_temporal_linear_fault = false;

    // Odom-Odom fault detection
    odom_temporal_fault = false;

    // Amcl-Other fault detection
    amcl_odom_linear_fault = false;
    amcl_odom_angular_fault = false;

    yCInfo(ROBOTNOTLOST_CONDITION) << "Initialization finished!";
}

double robotNotLost::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}

bool robotNotLost::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{

    return true;
}

// AMCL Callback
void robotNotLost::AmclPort::onRead(yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped &b)
{
    // Reset related faults
    parent_ref->amcl_temporal_angular_fault = false;
    parent_ref->amcl_temporal_linear_fault = false;
    parent_ref->amcl_odom_linear_fault = false;
    parent_ref->amcl_odom_angular_fault = false;

    // Get current pose and orientation from message
    yarp::math::Quaternion quaternion(b.pose.pose.orientation.x, b.pose.pose.orientation.y, b.pose.pose.orientation.z, b.pose.pose.orientation.w);

    std::array<double, 3> amcl_pose = {
        b.header.stamp,
        b.pose.pose.position.x,
        b.pose.pose.position.y};
    double avg_angle_difference = 0;
    double avg_distance_difference = 0;

    // Save all the previous poses and orientations
    parent_ref->amcl_orientations.push_back(quaternion);
    parent_ref->amcl_poses.push_back(amcl_pose);

    // Wait until there is enough data
    int amcl_orient_size = parent_ref->amcl_orientations.size();
    if (amcl_orient_size > 6)
    {

        for (int i = 1; i < 6; i++)
        {
            yarp::math::Quaternion last_quat = parent_ref->amcl_orientations[amcl_orient_size - i];
            yarp::math::Quaternion prev_last_qua = parent_ref->amcl_orientations[amcl_orient_size - i - 1].inverse();

            yarp::math::Quaternion quaternions_dif = last_quat * prev_last_qua.inverse();
            quaternions_dif.normalize();
            avg_angle_difference = quaternions_dif.toAxisAngle()[2];
        }

        avg_angle_difference /= 5.0;
        // If the angles difference is greater than about 7 degrees
        if (abs(avg_angle_difference) > 0.14)
        {
            parent_ref->amcl_temporal_angular_fault = true;
        }
    }

    int amcl_poses_size = parent_ref->amcl_poses.size();
    if (amcl_poses_size > 6)
    {
        for (int i = 1; i < 6; i++)
        {
            avg_distance_difference += hypot(parent_ref->amcl_poses[amcl_poses_size - i][1] - parent_ref->amcl_poses[amcl_poses_size - i - 1][1], parent_ref->amcl_poses[amcl_poses_size - i][2] - parent_ref->amcl_poses[amcl_poses_size - i - 1][2]);
        }

        avg_distance_difference /= 5.0;
        if (avg_distance_difference > 0.15)
        {
            parent_ref->amcl_temporal_linear_fault = true;
        }
    }

    // Keeps a separate list that updates with amcl, in order to have the rolling averages match. AMCL only updates when robot moves.
    if (parent_ref->odometry_velocities.size() > 1)
    {
        parent_ref->amcl_aligned_odometry_velocities.push_back(parent_ref->odometry_velocities[parent_ref->odometry_velocities.size() - 1]);
    }

    // Calculate the average velocity from amcl poses and timestamps
    double avg_linear_speed = 0.0;
    double avg_angular_speed = 0.0;
    double avg_elapsed_time = 0.0;
    int amcl_aligned_odom_vel_size = parent_ref->amcl_aligned_odometry_velocities.size();
    if (amcl_aligned_odom_vel_size > 6 && amcl_poses_size > 6)
    {
        for (int i = 1; i < 6; i++)
        {
            avg_linear_speed += parent_ref->amcl_aligned_odometry_velocities[amcl_aligned_odom_vel_size - i][1];
            avg_angular_speed += parent_ref->amcl_aligned_odometry_velocities[amcl_aligned_odom_vel_size - i][2];
            avg_elapsed_time += parent_ref->amcl_poses[amcl_poses_size - i][0] - parent_ref->amcl_poses[amcl_poses_size - i - 1][0];
        }

        avg_linear_speed /= 5.0;
        avg_elapsed_time /= 5.0;
        avg_angular_speed /= 5.0;

        // print("A-O Linear: " + str(abs((avg_distance_difference/avg_elapsed_time) - avg_linear_speed)))
        // print("A-O Angular: " + str(abs((avg_angle_difference/avg_elapsed_time) - avg_angular_speed)))

        // If the linear speed dif between odom and amcl is bigger than 0.05 m/s.
        if (abs((avg_distance_difference / avg_elapsed_time) - avg_linear_speed) > 0.05)
        {
            parent_ref->amcl_odom_linear_fault = true;
        }

        // If the angular speed dif between odom and amcl is bigger than 0.3 rad/s.
        if (abs((avg_angle_difference / avg_elapsed_time) - avg_angular_speed) > 0.3)
        {
            parent_ref->amcl_odom_angular_fault = true;
        }
    }
}

bool robotNotLost::hasCollided()
{
    if (cmd_velocities.size() > 3)
    {
        double cmd_vel_speed = cmd_velocities[cmd_velocities.size() - 3][1] - cmd_velocities[cmd_velocities.size() - 1][1];
        double cmd_vel_time = cmd_velocities[cmd_velocities.size() - 3][0] - cmd_velocities[cmd_velocities.size() - 1][0];
        double cmd_vel_acceleration = cmd_vel_speed / cmd_vel_time;

        // If the robot is NOT decelerating and has enough odom velocity and command velocity data, and has received command velocity data soon, then check for collision
        if (cmd_vel_acceleration > -0.1 && odometry_velocities.size() > 30 && abs(cmd_velocities[cmd_velocities.size() - 1][0] - yarp::os::Time::now()) < 0.2)
        {
            float slow_avg_acceleration = 0.0;
            float fast_avg_acceleration = 0.0;

            // Take the average of 5 velocities and timestamps in order to smooth out possible spikes (this is the average acceleration value of the last second)
            for (int i = 1; i < 31; i++)
            {
                slow_avg_acceleration += (odometry_velocities[cmd_velocities.size() - i][1] - odometry_velocities[cmd_velocities.size() - i - 1][1]) / (odometry_velocities[cmd_velocities.size() - i][0] - odometry_velocities[cmd_velocities.size() - i - 1][0]);
                if (i >= 25)
                {
                    fast_avg_acceleration += (odometry_velocities[cmd_velocities.size() - i][1] - odometry_velocities[cmd_velocities.size() - i - 1][1]) / (odometry_velocities[cmd_velocities.size() - i][0] - odometry_velocities[cmd_velocities.size() - i - 1][0]);
                }
            }

            slow_avg_acceleration /= 30;
            fast_avg_acceleration /= 5;

            if (abs(slow_avg_acceleration - fast_avg_acceleration) > 1.5)
            { // 1 m/s^2 acceleration
                yCDebug(ROBOTNOTLOST_CONDITION) << abs(slow_avg_acceleration - fast_avg_acceleration);
                return true;
            }
        }
    }
    return false;
}

void robotNotLost::OdomPort::onRead(yarp::rosmsg::nav_msgs::Odometry &b)
{
    // Reset related faults
    parent_ref->odom_temporal_fault = false;

    std::array<double, 3> velocity = {
        b.header.stamp,
        b.twist.twist.linear.x,
        b.twist.twist.angular.z};
    parent_ref->odometry_velocities.push_back(velocity);

    int odom_vel_size = parent_ref->odometry_velocities.size();
    if (odom_vel_size > 1)
    {
        // Max speed seems to be at 0.3 m/s so the threshold can be max_speed
        // Same for orientation with max angular orientation of about 30 degrees/s = 0.5 rad
        if (abs(parent_ref->odometry_velocities[odom_vel_size - 1][1] - parent_ref->odometry_velocities[odom_vel_size - 2][1]) > 0.3 || abs(parent_ref->odometry_velocities[odom_vel_size - 1][2] - parent_ref->odometry_velocities[odom_vel_size - 2][2]) > 0.523)
        {
            parent_ref->odom_temporal_fault = true;
        }
    }

    yarp::rosmsg::std_msgs::Bool p_msg;
    if (parent_ref->hasCollided())
    {
        p_msg.data = true;
        parent_ref->has_collided_pub.write(p_msg);
        return;
    }
    p_msg.data = false;
    parent_ref->has_collided_pub.write(p_msg);
}

void robotNotLost::CmdVelPort::onRead(yarp::rosmsg::geometry_msgs::Twist &b)
{
    std::array<double, 2> cmd_vel = {yarp::os::Time::now(), b.linear.x};
    parent_ref->cmd_velocities.push_back(cmd_vel);
}

// This is our main function. Will be called periodically every getPeriod() seconds
bool robotNotLost::updateModule()
{

    int total_faults = 0;

    if (amcl_temporal_angular_fault)
    {
        amcl_total_faults++;
        yCWarning(ROBOTNOTLOST_CONDITION) << "Low severity - amcl temporal angular consistency fault\t\t" << amcl_total_faults << "consecutive faults";
    }
    else if (amcl_temporal_linear_fault)
    {
        amcl_total_faults++;
        yCWarning(ROBOTNOTLOST_CONDITION) << "Low severity - amcl temporal linear consistency fault\t\t" << amcl_total_faults << "consecutive faults";
    }
    else if (amcl_odom_angular_fault)
    {
        amcl_total_faults++;
        yCWarning(ROBOTNOTLOST_CONDITION) << "Low severity - amcl-odometry angular consistency fault\t\t" << amcl_total_faults << "consecutive faults";
    }
    else if (amcl_odom_linear_fault)
    {
        amcl_total_faults++;
        yCWarning(ROBOTNOTLOST_CONDITION) << "Low severity - amcl-odometry linear consistency fault\t\t" << amcl_total_faults << "consecutive faults";
    }
    else
    {
        amcl_total_faults--;
        if (amcl_total_faults < 0)
        {
            amcl_total_faults = 0;
        }
    }

    // Odom Checks
    if (odom_temporal_fault)
    {
        odom_total_faults++;
        yCWarning(ROBOTNOTLOST_CONDITION) << "Low severity - odometry temporal consistency fault\t\t" << odom_total_faults << "consecutive faults";
    }
    else
    {
        odom_total_faults--;
        if (odom_total_faults < 0)
        {
            odom_total_faults = 0;
        }
    }

    // publish marker
    if (amcl_poses.size() > 0)
    {
        yarp::rosmsg::visualization_msgs::Marker robotMarker;

        robotMarker.header.frame_id = "map";
        robotMarker.header.stamp = yarp::os::Time::now();
        robotMarker.ns = "mobile_base_body_link";
        robotMarker.id = marker_count;

        marker_count++;

        robotMarker.type = yarp::rosmsg::visualization_msgs::Marker::CUBE;
        robotMarker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
        robotMarker.pose.position.x = amcl_poses[amcl_poses.size() - 1][1];
        robotMarker.pose.position.y = amcl_poses[amcl_poses.size() - 1][2];
        robotMarker.pose.position.z = 0.02;

        robotMarker.pose.orientation.x = 0;
        robotMarker.pose.orientation.y = 0;
        robotMarker.pose.orientation.z = 0;
        robotMarker.pose.orientation.w = 1.0;
        robotMarker.scale.x = 0.2;
        robotMarker.scale.y = 0.2;
        robotMarker.scale.z = 0.2;

        if (total_faults == 0)
        {
            robotMarker.color.r = 0.0;
            robotMarker.color.g = 1.0;
            robotMarker.color.b = 0.0;
            robotMarker.color.a = 0.5;
        }
        else if (total_faults > 5)
        {
            robotMarker.color.r = 1.0;
            robotMarker.color.g = 0.0;
            robotMarker.color.b = 0.0;
            robotMarker.color.a = 0.5;
        }
        else
        {
            robotMarker.color.r = 1.0;
            robotMarker.color.g = 1.0;
            robotMarker.color.b = 0.0;
            robotMarker.color.a = 0.5;
        }
        marker_array.markers.push_back(robotMarker);
        marker_pub.write(marker_array);
    }

    yarp::rosmsg::std_msgs::Int32 p_amcl, p_odom, p_total;
    p_amcl.data = amcl_total_faults;
    p_odom.data = odom_total_faults;
    p_total.data = total_faults;

    amcl_fault_pub.write(p_amcl);
    odom_fault_pub.write(p_odom);
    total_fault_pub.write(p_total);

    // Notify of results and reset
    if (amcl_total_faults == 0 && odom_total_faults == 0)
    {
        yCInfo(ROBOTNOTLOST_CONDITION) << "No fault";
        m_robotNotLost = true;
    }
    else
    {

        if (amcl_total_faults >= 3)
        {
            yCError(ROBOTNOTLOST_CONDITION) << "High severity - AMCL checks failed!";
        }

        if (odom_total_faults >= 3)
        {
            yCError(ROBOTNOTLOST_CONDITION) << "High severity - Odometry checks failed!";
        }

        total_faults = amcl_total_faults + odom_total_faults;

        if (total_faults >= 10)
        {
            m_robotNotLost = false;
            m_tourManager.sendError("LOCALIZATION_ERROR");
            yCError(ROBOTNOTLOST_CONDITION) << "Fail - Too many errors to continue!";
        }
        else
        {
            m_robotNotLost = true;
        }
    }

    return true;
}

bool robotNotLost::configure(yarp::os::ResourceFinder &rf)
{

    m_period = rf.check("period") ? rf.find("period").asInt32() : 0.5;

    if (!yarp::os::NetworkBase::checkNetwork())
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error! YARP Network is not initialized";
        return false;
    }

    if (!m_pBt.open(m_pBtName))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error! Cannot open YARP port" << m_pBtName;
        return false;
    }

    if (!this->yarp().attachAsServer(m_pBt))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error! Could not attach as server to the Bt";
        return false;
    }

    if (!marker_pub.topic("/consistencyMarker"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (publisher):"
                                        << "consistencyMarker";
        return false;
    }
    if (!has_collided_pub.topic("/collisionDetector"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (publisher):"
                                        << "collisionDetector";
        return false;
    }

    // Publish graph data
    if (!amcl_fault_pub.topic("/amcl_fault_pub"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (publisher):"
                                        << "amcl_fault_pub";
        return false;
    }
    if (!odom_fault_pub.topic("/odom_fault_pub"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (publisher):"
                                        << "odom_fault_pub";
        return false;
    }
    if (!total_fault_pub.topic("/total_fault_pub"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (publisher):"
                                        << "total_fault_pub";
        return false;
    }

    // Subscribers
    if (!amcl_port.topic("/amcl_pose"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (subscriber):"
                                        << "amcl_pose"; // Change to variable in the future
        return false;
    }
    amcl_port.useCallback();

    if (!odom_port.topic("/odometry"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (subscriber):"
                                        << "odometry";
        return false;
    }
    odom_port.useCallback();

    if (!cmd_vel_port.topic("/cmd_vel"))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error opening topic (subscriber):"
                                        << "cmd_vel";
        return false;
    }
    cmd_vel_port.useCallback();

    if (!m_tourManagerPort.open(m_tourManagerPortName))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error! Cannot the tourManagerRPC client port (%s)", m_tourManagerPortName.c_str();
        return false;
    }
    if (!m_tourManager.yarp().attachAsClient(m_tourManagerPort))
    {
        yCError(ROBOTNOTLOST_CONDITION) << "Error! Cannot attach the %s port as client", m_tourManagerPortName.c_str();
        return false;
    }

    yCInfo(ROBOTNOTLOST_CONDITION) << "Configuration done!";
    return true;
}
// Interrupt function.
bool robotNotLost::interruptModule()
{
    yCInfo(ROBOTNOTLOST_CONDITION) << "Interrupting your module, for port cleanup";
    m_pBt.close();
    // private_nh.shutdown();
    return true;
}
// Close function, to perform cleanup.
bool robotNotLost::close()
{
    yCInfo(ROBOTNOTLOST_CONDITION) << "Calling close function";
    return true;
}

SkillStatus robotNotLost::get_status()
{
    return m_status;
}

bool robotNotLost::start()
{
    return m_robotNotLost;
}

void robotNotLost::stop()
{
    yCError(ROBOTNOTLOST_CONDITION) << "Received a stop.";
}
