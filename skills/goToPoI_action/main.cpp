
/*
  * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
  * All rights reserved.
  *
  * This software may be modified and distributed under the terms of the
  * BSD-3-Clause license. See the accompanying LICENSE file for details.
  */

#include "bt_goToPoI_action.h"


int main(int argc, char * argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;

    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    std::string name = rf.check("name") ? rf.find("name").asString() : "GoToPoI_Act";

    GoToPoI_Act module(name);

    yInfo()<< "Configuring and starting module.";
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    if (!module.runModule(rf)) {
        yError() << "Error module did not start\n";
    }
    return 0;
}
