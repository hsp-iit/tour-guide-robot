#include <LinuxNetworkInteraction.h>
#include <yarp/os/LogStream.h>

std::vector<std::string> LinuxNetworkInteraction::getCurrentApName()
{
    // Get current connected AP name
    std::vector<std::string> iw_config_out;

    // Get iwconfig output
    FILE* pipe = popen("iwconfig 2>&1", "r");
    if(!pipe) {
        yError() << "Failed to execute iwconfig. Is iwconfig installed? Please run sudo apt update; sudo apt install wireless-tools";
        return iw_config_out;
    }

    // Parse iwconfig output to get Access Point name
    char buffer[128];
    while(fgets(buffer, 128, pipe) != NULL) {
        std::string line(buffer);
        iw_config_out.push_back(line);
    }

    return iw_config_out;
}

bool LinuxNetworkInteraction::roam(const std::string& ap_name)
{
    yWarning() << "Roaming capability is not implemented in LinuxNetworkInteraction";
    return true;
}