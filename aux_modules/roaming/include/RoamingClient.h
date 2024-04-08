#include <yarp/os/RFModule.h>
#include <yarp/os/Port.h>
#include <WpaSupplicantRoaming.h>

class RoamingClient: public yarp::os::RFModule
{
    public:
        RoamingClient();
        bool configure(yarp::os::ResourceFinder& rf) override;
        bool updateModule() override;
        bool interruptModule() override;
        double getPeriod() override;
        bool close() override;

    private:
        std::string m_name;
        std::string m_port_ap_name;
        std::string m_interface_name;
        std::string m_ssid;

        yarp::os::Port m_port_ap;

        WpaSupplicantRoaming roaming;
};