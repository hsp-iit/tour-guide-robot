#ifndef EARS_THREAD_HPP
#define EARS_THREAD_HPP

#include <mutex>
#include <string>
#include <iostream>

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
#include <yarp/sig/Sound.h>

class EarsThread : public yarp::os::PeriodicThread
{
public:
    EarsThread(yarp::os::ResourceFinder& _rf, std::string _moduleName, double _period, cv::Mat& _image, std::mutex& _mutex);

private:
    yarp::os::ResourceFinder& m_rf;
    yarp::os::BufferedPort<yarp::sig::Sound > m_audioRecPort;
    yarp::os::BufferedPort<yarp::sig::AudioRecorderStatus> m_audioStatusPort;
    std::mutex&             m_drawing_mutex;
    std::recursive_mutex    m_methods_mutex;
    std::string             m_imagePath;
    cv::Mat&                m_face;
    std::string             m_moduleName;

    cv::Mat                 m_earBar;
    cv::Mat                 m_blackBar;

    cv::Scalar              m_earsDefaultColor = cv::Scalar(0, 128, 0);
    cv::Scalar              m_earsCurrentColor = cv::Scalar(0, 128, 0);

    bool m_doBars = false;
    bool m_drawEnable = true;
    bool m_micIsEnabled = false;

    // Offset values for placing stuff and size
    int barWidth = 1;
    int earBarL0_x = 1;
    int earBarL0_y = 6;
    int earBar0_minLen = 3;
    int earBar0_maxLen = 18;
    int earBarL1_x = 3;
    int earBarL1_y = 6;
    int earBar1_minLen = 4;
    int earBar1_maxLen = 19;
    int earBarR0_x;    // Shall be constant as well, but cannot determine size at compile time
    int earBarR1_x;
    int earBarR0_y = 6;
    int earBarR1_y = 6;

    // Actual size of bars, changing
    int earBar0_len = 14;
    int earBar1_len = 16;

    bool updateBars(float percentage);
    void clearWithBlack();

public:
    bool threadInit()  override;
    void threadRelease()  override;
    void afterStart(bool s)  override;
    void run() override;

    void activateBars (bool activate);
    void resetToDefault();

    void setColor(float vr, float vg, float vb);
    void enableDrawing(bool activate);
};

#endif
