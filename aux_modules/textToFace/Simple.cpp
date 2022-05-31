/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Simple.h"
#include <yarp/os/Bottle.h>
#include <yarp/os/Things.h>
#include <yarp/os/Log.h>
#include <yarp/sig/Sound.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
YARP_LOG_COMPONENT(MICROPHONE_FILTER, "behavior_tour_robot.aux_modules.textToFace", yarp::os::Log::TraceType)

bool MicrophoneFilter::create(const yarp::os::Property &options)
{
    yCDebug(MICROPHONE_FILTER, "created!\n");
    yCDebug(MICROPHONE_FILTER, "I am attached to the %s\n",
            (options.find("sender_side").asBool()) ? "sender side" : "receiver side");
    channel = (options.check("channel") ? options.find("channel").asInt8() : 0);
    yCDebug(MICROPHONE_FILTER, "channel number : %d", channel);
    return true;
}

void MicrophoneFilter::destroy()
{
    yDebug("destroyed!\n");
}

bool MicrophoneFilter::setparam(const yarp::os::Property &params)
{
    channel = params.find("channel").asInt8();
    return false;
}

bool MicrophoneFilter::getparam(yarp::os::Property &params)
{
    params.put("channel", channel);
    return false;
}

bool MicrophoneFilter::accept(yarp::os::Things &thing)
{
    yarp::sig::Sound *bt = thing.cast_as<yarp::sig::Sound>();
    if (bt == NULL)
    {
        yCWarning(MICROPHONE_FILTER, "SimpleMonitorObject: expected type Sound but got wrong data type!\n");
        return false;
    }
    yCWarning(MICROPHONE_FILTER, "0");

    return true;
}

yarp::os::Things &MicrophoneFilter::update(yarp::os::Things &thing)
{
    yarp::sig::Sound *s = thing.cast_as<yarp::sig::Sound>();
    if (s == NULL)
    {
        yCWarning(MICROPHONE_FILTER, "SimpleMonitorObject: expected type Bottle but got wrong data type!\n");
        return thing;
    }
    s2.clear();
    //s2=s->getChannel(1);

    s2 = s->extractChannelAsSound(channel);
    th.setPortWriter(&s2);
    yCWarning(MICROPHONE_FILTER)<< channel<<"\n";
    return th;
}

void MicrophoneFilter::trig()
{
}
