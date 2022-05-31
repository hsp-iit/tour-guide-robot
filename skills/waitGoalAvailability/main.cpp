/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#include "bt_waitGoalAvailability.h"

int main(int argc, char *argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;

    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    std::string name = rf.check("name") ? rf.find("name").asString() : "waitGoalAvailability";
    waitGoalAvailability module(name);

    std::cout << "Configuring and starting module.\n";
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    if (!module.runModule(rf))
    {
        std::cerr << "Error module did not start\n";
    }

    std::cout << "Main returning..." << '\n';
    return 0;
}
