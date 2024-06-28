#include <RoamingServer.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>

// #include <yarp/dev/Nav2DInterfaces.h>

#include <catch2/catch_test_macros.hpp>

#include <iostream>

#include <FakeNetworkInteraction.h>

void increment_tests_skipped();

// #define YARP_SKIP_TEST(...) \
// { \
//     increment_tests_skipped(); \
//     FAIL(__VA_ARGS__); \
// }

// #define YARP_REQUIRE_PLUGIN(name, type) \
// { \
//     bool has_plugin = yarp::os::YarpPluginSelector::checkPlugin(name, type); \
//     if (!has_plugin) { \
//         YARP_SKIP_TEST("Required plugin is missing: " << type << " - " << name); \
//     } \
// }

TEST_CASE("Roaming::functionalities", "[Roaming]")
{
    // YARP_REQUIRE_PLUGIN("fakeLocalizer", "device");
    // YARP_REQUIRE_PLUGIN("fakeNavigation", "device");
    // YARP_REQUIRE_PLUGIN("map2DStorage", "device");
    // YARP_REQUIRE_PLUGIN("navigation2D_nwc_yarp", "device");

    // yarp::os::Network::setLocalMode(true);

    // Open the devices
    
    // Open fakeLocalizer and localizer nws
    yarp::dev::PolyDriver dd_loc;
    yarp::dev::PolyDriver dd_loc_nws;

    yarp::os::Property p_loc;
    yarp::os::Property p_loc_nws;
    p_loc.put("device", "fakeLocalizer");
    p_loc_nws.put("device", "localization2D_nws_yarp");

    REQUIRE(dd_loc.open(p_loc));
    REQUIRE(dd_loc_nws.open(p_loc_nws));
    
    // Attach fakelocalizer to localizer nws
    yarp::dev::WrapperSingle* ww_nws_loc;
    dd_loc_nws.view(ww_nws_loc);
    REQUIRE(ww_nws_loc);
    ww_nws_loc->attach(&dd_loc);

    // Open map server
    yarp::dev::PolyDriver dd_map;
    yarp::os::Property p_map;
    p_map.put("device", "map2DStorage");
    REQUIRE(dd_map.open(p_map));
    
    yarp::dev::PolyDriver dd_mapserver;
    yarp::os::Property p_mapserver;
    p_mapserver.put("device", "map2D_nws_yarp");
    REQUIRE(dd_mapserver.open(p_mapserver));
    
    yarp::dev::WrapperSingle* ww_nws;
    dd_mapserver.view(ww_nws);
    REQUIRE(ww_nws);
    ww_nws->attach(&dd_map);

    // Open fakeNavigation
    yarp::dev::PolyDriver dd_nav;
    yarp::os::Property p_nav;
    p_nav.put("device", "fakeNavigation");
    REQUIRE(dd_nav.open(p_nav));

    yarp::dev::PolyDriver dd_navserver;
    yarp::os::Property p_navserver;
    p_navserver.put("device", "navigation2D_nws_yarp");
    REQUIRE(dd_navserver.open(p_navserver));

    yarp::dev::WrapperSingle* ww_nws_nav;
    dd_navserver.view(ww_nws_nav);
    REQUIRE(ww_nws_nav);
    ww_nws_nav->attach(&dd_nav);

    // Set up fake AP location
    yarp::dev::PolyDriver dd_nav_client;
    yarp::os::Property options;
    options.put("device", "navigation2D_nwc_yarp");
    options.put("local", "/roaming_tests/navigation2D_nwc_yarp");
    options.put("navigation_server", "/navigation2D_nws_yarp");
    options.put("map_locations_server", "/map2D_nws_yarp");
    options.put("localization_server", "/localization2D_nws_yarp");

    yarp::dev::Nav2D::INavigation2D *m_navigation;
    yarp::dev::Nav2D::ILocalization2D *m_localization;

    REQUIRE(dd_nav_client.open(options));
    
    dd_nav_client.view(m_navigation);
    dd_nav_client.view(m_localization);

    yarp::dev::Nav2D::Map2DLocation robot_loc;
    m_localization->getCurrentPosition(robot_loc); //robot_loc.map_id = test

    std::cout << "fbrand: robot loc x" << robot_loc.x << "\n";
    std::cout << "fbrand: robot loc y" << robot_loc.y << "\n";

    std::string fake_ap1_location{"area_1"}; //Just a fake MAC address
    std::string fake_ap1_mac{"X0:00:XX:X0:0X:00"};
    yarp::dev::Nav2D::Map2DLocation ap_loc(robot_loc.map_id,1.0,0.0,0.0,"ap1");
    m_navigation->storeLocation(fake_ap1_location,ap_loc); //Just a fake MAC address
    
    std::string fake_ap2_location{"area_2"};
    std::string fake_ap2_mac{"X1:0X:XX:X1:0X:00"};
    yarp::dev::Nav2D::Map2DLocation ap2_loc(robot_loc.map_id,2.0,0.0,0.0,"ap2");
    m_navigation->storeLocation(fake_ap2_location,ap2_loc);

    // Instantiate the FakeNetworkInteraction to read iwconfig dump from files
    auto inet = FakeNetworkInteraction();
    RoamingServer roaming("roaming",inet);

    SECTION("Configure roaming module")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
    }

    SECTION("Check isAP")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
        REQUIRE(roaming.isAP(fake_ap1_mac));
    }

    SECTION("Check AP location")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
        auto fake_ap_location = roaming.getApPosition(fake_ap1_mac);
        REQUIRE(fake_ap_location.has_value());
        auto fake_ap_location_value = fake_ap_location.value();
        REQUIRE(fake_ap_location_value.x < 1.1);
        REQUIRE(fake_ap_location_value.x > 0.9); //float comparison
        REQUIRE(fake_ap_location_value.y < 0.1); 
        REQUIRE(fake_ap_location_value.y > -0.1); //float comparison
    }

    SECTION("Check distance to AP")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
        REQUIRE(roaming.distanceToAP(fake_ap1_mac) < 1.1);
        REQUIRE(roaming.distanceToAP(fake_ap1_mac) > 0.9); //float comparison
    }

    SECTION("Check getCurrentAPName when iwconfig connected")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
        inet.loadIwConfigDump("/workspaces/tour-guide-robot/aux_modules/roaming/test/data/iwconfig_dump_connected.txt");
        bool check{roaming.getCurrentApName() == "X1:0X:XX:X1:0X:00"};
        REQUIRE(check);
    }

    SECTION("Check getCurrentAPName when iwconfig disconnected")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
        inet.loadIwConfigDump("/workspaces/tour-guide-robot/aux_modules/roaming/test/data/iwconfig_no_connection.txt");
        bool check{!roaming.getCurrentApName().has_value()};
        REQUIRE(check);
    }

    SECTION("Check getBestAp")
    {
        yarp::os::ResourceFinder rf;
        rf.setDefault("loc_to_ap_map","/workspaces/tour-guide-robot/aux_modules/roaming/test/data/locations_ap_map.txt");
        REQUIRE(roaming.configure(rf));
        inet.loadIwConfigDump("/workspaces/tour-guide-robot/aux_modules/roaming/test/data/iwconfig_dump_connected.txt");
        bool check{roaming.getBestAP() == fake_ap1_mac};
        REQUIRE(check);
    }

    // yarp::os::Network::setLocalMode(false);
}
