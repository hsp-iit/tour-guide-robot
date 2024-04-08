#include <string>
#include <wpa_ctrl.h>

class WpaSupplicantRoaming
{
    public:
        bool roam(const std::string& if_name, const std::string& ssid, const std::string& ap_name);
    private:
        std::string wpa_request(wpa_ctrl* ctrl, const std::string& cmd);
};