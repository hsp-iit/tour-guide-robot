/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "eyeContactManager.h"

int main(int argc, char *argv[])
{
  // initialize yarp network
  yarp::os::Network yarp;

  // prepare and configure the resource finder
  yarp::os::ResourceFinder rf;
  rf.configure(argc, argv);

  eyeContactManager manager("eyeContactManager");

  if (!manager.runModule(rf))
  {
    yError() << "Error module eyeContactManager did not start";
  }
  return 0;
}
