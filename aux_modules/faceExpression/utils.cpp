#include "utils.hpp"

bool getPath(yarp::os::ResourceFinder& m_rf, std::string& m_imagePath)
{
    if (!m_rf.check("path"))
    {
        std::string imagePath_test = m_rf.findFileByName("images/blink_1.bmp");
        size_t last = imagePath_test.rfind("/");
        m_imagePath = imagePath_test.substr(0, last);
    }
    else
    {
        m_imagePath = m_rf.find("path").asString();
    }
    return true;
}