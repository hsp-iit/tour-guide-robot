#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <yarp/os/ResourceFinder.h>

#define FACE_WIDTH      80
#define FACE_HEIGHT     32

bool getPath(yarp::os::ResourceFinder& m_rf, std::string& m_imagePath);

#endif