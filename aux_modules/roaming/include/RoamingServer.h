#include <yarp/os/RFModule.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Port.h>

#include <optional>
#include <map>

#include <INetworkInteraction.h>

using LocationApMap = std::map<std::string,std::string>;

class RoamingServer : public yarp::os::RFModule
{
public:
    RoamingServer(std::string name);
    RoamingServer(std::string name, INetworkInteraction& inet);
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool close();
    virtual bool interruptModule();

    bool roam(const std::string &ap_name);

    // APs
    const std::vector<std::string> getAPList() const;
    const std::optional<yarp::dev::Nav2D::Map2DLocation> getApPosition(const std::string &ap_name) const;
    const std::optional<std::string> getCurrentApName() const;
    const bool isAP(const std::string &location_name) const;
    std::pair<LocationApMap,LocationApMap> getLocationAPMaps(std::ifstream& map_file) const;
    const double distanceToAP(const std::string& ap_name) const;
    const std::string getBestAP() const;

private:
    std::string m_name;
    std::string m_roaming_port_name;
    std::string m_locations_to_ap_file_name;
    yarp::os::Port m_roaming_port;

    yarp::dev::Nav2D::INavigation2D *m_navigation;
    yarp::dev::Nav2D::ILocalization2D *m_localization;
    yarp::dev::Nav2D::IMap2D *m_map;
    yarp::dev::PolyDriver m_ddNav;

    INetworkInteraction& m_net;
    
    std::vector<std::string> m_locations_list;
    std::map<std::string,std::string> m_location_to_ap;
    std::map<std::string,std::string> m_ap_to_location;
    std::string m_map_name;

    // Configuration
    bool configureInterfaces();

    // Roaming 
    yarp::dev::Nav2D::Map2DLocation m_robot_position;
    bool checkRoamingCondition(const double threshold = 5.0);
};