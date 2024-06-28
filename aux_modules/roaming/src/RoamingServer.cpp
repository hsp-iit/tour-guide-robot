#include <RoamingServer.h>
#include <yarp/os/LogStream.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <algorithm>

RoamingServer::RoamingServer(std::string name, INetworkInteraction& inet):
    m_name{name},
    m_net{inet}
{
    m_navigation = nullptr;
}

bool RoamingServer::configure(yarp::os::ResourceFinder &rf)
{
    // Return help if needed
    if(rf.check("help") || !rf.check("loc_to_ap_map"))
    {
        yInfo() << "RoamingServer options:";
        yInfo() << "\t--loc_to_ap_map: file containing location to AP map";
        return false;
    }

    m_locations_to_ap_file_name = rf.check("loc_to_ap_map", yarp::os::Value("locations_ap_map.txt")).asString();
    
    // Open file
    std::ifstream locations_to_ap_map_file(m_locations_to_ap_file_name);
    if(!locations_to_ap_map_file.is_open())
    {
        yError() << "Failed to open file" << m_locations_to_ap_file_name;
        return false;
    }

    // Configure interfaces to external services
    configureInterfaces();

    // Get ap and locations maps and list
    const auto& [location_to_ap, ap_to_location] = getLocationAPMaps(locations_to_ap_map_file);
    m_location_to_ap = location_to_ap;
    m_ap_to_location = ap_to_location;

    m_roaming_port_name = "/" + m_name + "/ap:o";
    m_roaming_port.open(m_roaming_port_name);

    yDebug() << "Roaming port opened at" << m_roaming_port_name;

    // Get APs from map
    try 
    {
        m_locations_list = getAPList();
    }
    catch(const std::runtime_error& e)
    {
        yError() << e.what();
        return false;
    }
    
    yDebug() << "APs found in map" << m_locations_list.size();

    if(m_locations_list.size() == 0) {
        yError() << "None of the locations defined in the mapping file are found in the map server";
        return false;
    }

    // Get robot position
    m_navigation->getCurrentPosition(m_robot_position);
    m_map_name = m_robot_position.map_id;

    return true;
}

bool RoamingServer::configureInterfaces()
{
    yarp::os::Property options;
    options.put("device", "navigation2D_nwc_yarp");
    options.put("local", "/" + m_name + "/navigation2D_nwc_yarp");
    options.put("navigation_server", "/navigation2D_nws_yarp");
    options.put("map_locations_server", "/map2D_nws_yarp");
    options.put("localization_server", "/localization2D_nws_yarp");

    if (!m_ddNav.open(options))
    {
        yError() << "Failed to open navigation2Dclient device";
        return false;
    }

    if (!m_ddNav.view(m_localization))
    {
        yError() << "Failed to get ILocalization2D interface";
        return false;
    }

    if (!m_ddNav.view(m_navigation))
    {
        yError() << "Failed to get INavigation2D interface";
        return false;
    }

    return true;

}

bool RoamingServer::close()
{
    m_roaming_port.close();
    m_ddNav.close();
    return true;
}

bool RoamingServer::updateModule()
{
    // Do something with m_navigation
    if (checkRoamingCondition())
    {
        std::string ap_name = getBestAP();
        if (ap_name != "")
        {
            yDebug() << "Roaming to AP" << ap_name;
            if (!roam(ap_name))
            {
                yError() << "Roaming to AP failed";
            }
        }
    }

    return true;
}

bool RoamingServer::interruptModule()
{
    m_roaming_port.interrupt();
    return true;
}

double RoamingServer::getPeriod()
{
    return 1.0;
}

const double RoamingServer::distanceToAP(const std::string& ap_name) const {
    
    // Get AP position from map
    std::optional<yarp::dev::Nav2D::Map2DLocation> ap_position = getApPosition(ap_name);

    // Calculate distance between robot and AP
    if(!ap_position.has_value()) {
        yError() << "AP" << ap_name << "not found in map";
        return false;
    }

    double distance = sqrt(pow(m_robot_position.x - ap_position->x, 2) + pow(m_robot_position.y - ap_position->y, 2));
    yDebug() << "Distance to AP" << ap_name << "is" << distance;\

    return distance;

}

bool RoamingServer::checkRoamingCondition(const double threshold)
{
    // Get current connected AP name
    std::optional<std::string> current_ap_name = getCurrentApName();

    if(!current_ap_name.has_value())
    {
        yInfo() << "No current connected AP. Are you connected to internet?";
        return true;
    }

    // Get robot position from localization
    m_localization->getCurrentPosition(m_robot_position);

    // Get AP position from map
    double distance = distanceToAP(current_ap_name.value());

    // If distance is greater than threshold, return true
    if (distance > threshold)
    {
        return true;
    }

    return false;
}

const std::string RoamingServer::getBestAP() const {

    std::string best_ap = "";

    auto current_ap_name = getCurrentApName();

    double min_dist = -std::numeric_limits<double>::infinity();

    if(!current_ap_name.has_value())
    {
        yWarning() << "No current connected AP. Are you connected to internet?";
    }
    else
    {
        min_dist = distanceToAP(current_ap_name.value());
    }

    for (const auto& [ap_mac_addr,ap_location] : m_ap_to_location) {
        if (double dist = distanceToAP(ap_mac_addr) < min_dist) {
            best_ap = ap_mac_addr;
            min_dist = dist;
        }
    }

    return best_ap;
}


bool RoamingServer::roam(const std::string &ap_name)
{
    if (checkRoamingCondition()) {
        
        yDebug() << "Roaming to AP" << ap_name;

        // Get next best AP from map
        std::string next_ap = getBestAP();

        // Roam to next AP
        yarp::os::Bottle roaming_ap;
        roaming_ap.addString(next_ap);
        m_roaming_port.write(roaming_ap);
    }

    return true;
}

const std::optional<yarp::dev::Nav2D::Map2DLocation> RoamingServer::getApPosition(const std::string &ap_name) const
{

    yDebug() << "getApPosition" << ap_name;

    // Get AP position from map
    std::vector<yarp::dev::Nav2D::Map2DLocation> locations;
    
    if(!m_navigation)
    {
        yError() << "Navigation interface not available";
        return {};
    }

    for (const auto& [ap_location,ap_mac_address] : m_location_to_ap)
    {
        yDebug() << "AP location" << ap_location << "AP MAC address" << ap_mac_address;
        
        if(ap_mac_address == ap_name)
        {
            yarp::dev::Nav2D::Map2DLocation location;
            m_navigation->getLocation(ap_location, location);
            if(location.map_id == m_map_name)
            {
                return location;
            }
        }
    }

    return {};
}

const std::optional<std::string> RoamingServer::getCurrentApName() const
{

    std::string current_ap_name;

    std::vector<std::string> lines = m_net.getCurrentApName();

    for(const auto& line: lines)
    {
        if(line.find("Access Point: Not-Associated") != std::string::npos) {
            yWarning() << "You are not connected to the internet!";
            return {};
        }
        if (line.find("Access Point") != std::string::npos) {
            std::string::size_type start = line.find("Access Point:") + 14;
            std::string::size_type end = line.find(" ", start);
            current_ap_name = line.substr(start, end - start);
            return current_ap_name;
        }
    }

    return {};

}

const std::vector<std::string> RoamingServer::getAPList() const {

    std::vector<std::string> locations;
    std::vector<std::string> aps;

    // Get all locations from map
    if(!m_navigation)
    {
        yError() << "Navigation interface is not available";
        throw std::runtime_error("Navigation interface is not available");
    }

    m_navigation->getLocationsList(locations);

    // For every location in the mapping, get the mac address and check that it's well defined
    for (const auto& [ap_location,ap_mac_add]: m_location_to_ap)
    {
        if(std::find(locations.begin(),locations.end(),ap_location) != locations.end())
        {
            if(this->isAP(ap_mac_add))
            {
                aps.push_back(ap_mac_add);
            }
        }
    }

    return aps;
}

std::pair<LocationApMap,LocationApMap> RoamingServer::getLocationAPMaps(std::ifstream& map_file) const
{
    LocationApMap location_to_ap;
    LocationApMap ap_to_location;

    std::string line;
    while (std::getline(map_file, line))
    {
        std::string location;
        std::string ap;
        std::istringstream iss(line);
        iss >> location >> ap;
        location_to_ap.insert_or_assign(location,ap);
        ap_to_location.insert_or_assign(ap,location);
    }

    return {location_to_ap,ap_to_location};
}

const bool RoamingServer::isAP(const std::string &ap_name) const {

    bool is_ap = false;

    // MAC address has 17 characters
    if(ap_name.size() != 17)
    {
        return false;
    }

    // If it looks like a MAC address, then it is a MAC address
    is_ap = ap_name[2] == ':';
    is_ap = ap_name[5] == ':';
    is_ap = ap_name[8] == ':';
    is_ap = ap_name[11] == ':';
    is_ap = ap_name[14] == ':';
    
    return is_ap;
}