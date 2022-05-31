/*
 * Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <iostream>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

#include "faceExpressionImage.hpp"

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setDefaultContext("faceExpressionImage");
    rf.setDefaultConfigFile("faceExpressionImage.ini");
    rf.configure(argc, argv);

    if (rf.check("help"))
    {
        yInfo("Possible options: ");
        yInfo("'robot <name>' the robot name for remote connection.");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    FaceExpressionImageModule mod;
    mod.setName("faceExpressionImage");

    return mod.runModule(rf);
}
