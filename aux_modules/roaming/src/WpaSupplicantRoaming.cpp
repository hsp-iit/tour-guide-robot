#include <WpaSupplicantRoaming.h>
#include <wpa_ctrl.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <iostream>

bool WpaSupplicantRoaming::roam(const std::string& if_name, const std::string& ssid, const std::string& bssid)
{
    std::string wpa_if = "/var/run/wpa_supplicant/"+if_name;
    auto* wpa_ctrl = wpa_ctrl_open(wpa_if.c_str());
    
    if(!wpa_ctrl)
    {
        yError() << "Could not open network interface" << if_name << ", aborting.";
        return false;
    }

    //TODO: there might be a bug here if ssid contains spaces
    const std::string bssid_cmd = "BSSID " + ssid + " " + bssid;
    const std::string bssid_reply = wpa_request(wpa_ctrl,bssid_cmd);

    if(bssid_reply != "OK")
    {
        yError() << "CMD:" << bssid_cmd << "failed. Aborting.";
        return false;
    }
    else
    {
        yDebug() << "CMD:" << bssid_cmd << "successful.";
    }

    const std::string reassociate_cmd = "REASSOCIATE";
    const auto reassociate_reply = wpa_request(wpa_ctrl,reassociate_cmd);
    if(reassociate_reply != "OK")
    {
        yError() << "CMD:" << reassociate_cmd << "failed. Aborting.";
        return false;
    }
    else
    {
        yDebug() << "CMD:" << reassociate_cmd << "successful.";
    }

    return true;
}

std::string WpaSupplicantRoaming::wpa_request(wpa_ctrl* ctrl, const std::string& cmd)
{
    char buf[4096];
    std::size_t cmd_reply_len = sizeof(buf) - 1;
    if(wpa_ctrl_request(ctrl,cmd.c_str(),cmd.size(),buf,&cmd_reply_len,NULL) != 0)
    {
        yError() << "Failed to send/receivce command to/from wpa_supplicant";
        return "";
    }
    std::string cmd_reply(buf,cmd_reply_len-1);
    return cmd_reply;
}