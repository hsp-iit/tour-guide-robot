
/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef NOTLOST_H
#define NOTLOST_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Node.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogComponent.h>
#include <yarp/math/Quaternion.h>
#include <yarp/math/Math.h>
#include <iostream>
#include <atomic>
#include <Skill_request.h>
#include <math.h>
#include <tourManagerRPC.h>

// Diagnostics
#include <yarp/rosmsg/geometry_msgs/PoseWithCovarianceStamped.h>
#include <yarp/rosmsg/geometry_msgs/PoseStamped.h>
#include <yarp/rosmsg/geometry_msgs/Twist.h>
#include <yarp/rosmsg/visualization_msgs/Marker.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <yarp/rosmsg/nav_msgs/Odometry.h>
#include <yarp/rosmsg/std_msgs/Bool.h>
#include <yarp/rosmsg/std_msgs/Int32.h>

class robotNotLost : public yarp::os::RFModule, public Skill_request
{
public:
  robotNotLost(std::string name);
  double getPeriod();
  // This is our main function. Will be called periodically every getPeriod() seconds
  bool updateModule();
  bool configure(yarp::os::ResourceFinder &rf);
  // Interrupt function.
  bool interruptModule();
  // Close function, to perform cleanup.
  bool close();
  virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
  SkillStatus get_status() override;
  bool start() override;
  void stop() override;
  bool hasCollided();
  class AmclPort : public yarp::os::Subscriber<yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped>
  {
  public:
    using yarp::os::Subscriber<yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped>::onRead;
    void onRead(yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped &b) override;
    robotNotLost *parent_ref;
  };
  class OdomPort : public yarp::os::Subscriber<yarp::rosmsg::nav_msgs::Odometry>
  {
  public:
    using yarp::os::Subscriber<yarp::rosmsg::nav_msgs::Odometry>::onRead;
    void onRead(yarp::rosmsg::nav_msgs::Odometry &b) override;
    robotNotLost *parent_ref;
  };
  class CmdVelPort : public yarp::os::Subscriber<yarp::rosmsg::geometry_msgs::Twist>
  {
  public:
    using yarp::os::Subscriber<yarp::rosmsg::geometry_msgs::Twist>::onRead;
    void onRead(yarp::rosmsg::geometry_msgs::Twist &b) override;
    robotNotLost *parent_ref;
  };

  // Subscribers
  AmclPort amcl_port;
  OdomPort odom_port;
  CmdVelPort cmd_vel_port;

  // Internal member functions
private:
  std::string m_name;
  std::string m_pBtName;
  double m_period;
  bool m_robotNotLost;
  yarp::os::RpcServer m_pBt;
  yarp::os::Node private_nh;
  std::atomic<SkillStatus> m_status;

  tourManagerRPC m_tourManager;
  std::string m_tourManagerPortName;
  yarp::os::Port m_tourManagerPort;

  // Visualisations
  yarp::rosmsg::visualization_msgs::MarkerArray marker_array;
  int marker_count = 0;

  // Data vectors
  std::vector<std::array<double, 3>> amcl_poses;
  std::vector<yarp::math::Quaternion> amcl_orientations;
  std::vector<std::array<double, 3>> odometry_velocities;
  std::vector<std::array<double, 3>> amcl_aligned_odometry_velocities;
  std::vector<std::array<double, 2>> cmd_velocities;

  // Total number of accumulated faults
  int amcl_total_faults;
  int odom_total_faults;

  // Amcl-Amcl fault detection
  bool amcl_temporal_angular_fault;
  bool amcl_temporal_linear_fault;

  // Odom-Odom fault detection
  bool odom_temporal_fault;

  // Amcl-Other fault detection
  bool amcl_odom_linear_fault;
  bool amcl_odom_angular_fault;

  yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray> marker_pub;
  yarp::os::Publisher<yarp::rosmsg::std_msgs::Bool> has_collided_pub;

  // Publish graph data
  yarp::os::Publisher<yarp::rosmsg::std_msgs::Int32> amcl_fault_pub;
  yarp::os::Publisher<yarp::rosmsg::std_msgs::Int32> odom_fault_pub;
  yarp::os::Publisher<yarp::rosmsg::std_msgs::Int32> total_fault_pub;
};

#endif // NOTLOST_H
