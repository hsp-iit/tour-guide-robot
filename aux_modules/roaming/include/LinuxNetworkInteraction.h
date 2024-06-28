#ifndef LINUXNETWORKINTERACTION_H_
#define LINUXNETWORKINTERACTION_H_

#include <INetworkInteraction.h>

class LinuxNetworkInteraction : public INetworkInteraction 
{

    public:
        LinuxNetworkInteraction() {};
        std::vector<std::string> getCurrentApName() override;
        bool roam(const std::string& ap_name) override;
};

#endif