#include <memory>

#include "VADMsgs.h"
#include "Detector.h"

class VADServer : public VADMsgs
{
    std::shared_ptr<Detector> m_detector;
 
public:
    VADServer(std::shared_ptr<Detector> detector);
    bool run_inference(const bool active) override;
};