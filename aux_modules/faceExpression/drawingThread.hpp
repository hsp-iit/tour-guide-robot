#ifndef DRAWING_THREAD_HPP
#define DRAWING_THREAD_HPP

#include <mutex>
#include <string>
#include <iostream>
#include "utils.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/core/mat.hpp>

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/AudioRecorderStatus.h>
#include <yarp/sig/AudioPlayerStatus.h>

class DrawingThread : public yarp::os::PeriodicThread
{
public:
    DrawingThread(yarp::os::ResourceFinder& _rf, std::string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex);

private:
    yarp::os::ResourceFinder& m_rf;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> m_imageOutPort;
    std::mutex&             m_mutex;
    cv::Mat& m_image;
    std::string m_moduleName;

public:
    bool threadInit()  override;
    void threadRelease()  override;
    void afterStart(bool s)  override;
    void run() override;

    void blackReset();
};

#endif
