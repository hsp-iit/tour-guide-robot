#ifndef INETWORKINTERACTION_H_
#define INETWORKINTERACTION_H_

#include <string>
#include <vector>

class INetworkInteraction
{
    public:
        virtual ~INetworkInteraction() {};
        virtual std::vector<std::string> getCurrentApName() = 0;
        virtual bool roam(const std::string& ap_name) = 0;
};

#endif