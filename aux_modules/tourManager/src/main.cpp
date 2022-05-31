#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <tourManager.h>
#include <yarp/os/LogStream.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    std::string name = rf.check("name") ? rf.find("name").asString() : "TourManager";
    std::string nameJSONTours = rf.check("nameJSONTours") ? rf.find("nameJSONTours").asString() : "tours.json";
    std::string nameJSONMovements = rf.check("nameJSONMovements") ? rf.find("nameJSONMovements").asString() : "movements.json";
    std::string tourName = rf.check("tourName") ? rf.find("tourName").asString() : "TOUR_SIM_GAM";
    std::string pathJSONTours = rf.findFileByName(nameJSONTours);
    std::string pathJSONMovements = rf.findFileByName(nameJSONMovements);

    TourManager manager(name, pathJSONTours, pathJSONMovements, tourName);
    yInfo() << "Configuring and starting module...";
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    if (!manager.runModule(rf))
    {
        yError() << "Error module did not start!";
    }

    yDebug() << "Main returning...";
    return EXIT_SUCCESS;
}