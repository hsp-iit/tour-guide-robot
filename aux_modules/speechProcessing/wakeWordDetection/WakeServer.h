#include <memory>

#include "WakeMsgs.h"
#include "AudioCallback.h"

class WakeServer : public WakeMsgs
{
    std::shared_ptr<AudioCallback> m_detector;
 
public:
    WakeServer(std::shared_ptr<AudioCallback> audioCallback);
    void stop() override;
};