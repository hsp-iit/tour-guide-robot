/*
*************************************************************************************************************************************************************
This software and all the related documentation that are transmitted as a final report constitute Foregrund of the Research Agreement between IIT and Konica
and their use and exploitation are subject to the limitations contained therein.
*************************************************************************************************************************************************************
*/

#include "crowdDetector.h"


YARP_LOG_COMPONENT(CROWD_DETECTOR, "behavior_tour_robot.aux_modules.crowd_detector", yarp::os::Log::TraceType)

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

    // config PARAMETERS:
    // name                 -> name of the module
    // targetFrame         -> set the target frame to which bodies coordinates detected are referred
    // m_area_radius          -> set the radius around the POI to count people [m]
    // remoteTC remoteTC   -> remote port required by FrameTransformClient::open()


     double MyModule::getPeriod()
     {
         // module periodicity (seconds), called implicitly by the module.
         return 0.5;
     }
     // This is our main function. Will be called periodically every getPeriod() seconds
     bool MyModule::updateModule()
     {
         // PRINT RUNNING MODULE
         count++;
         std::cout << "[" << count << "]" << " updateModule..." << '\n';


         // READ HUMAN PRESENCE AND COUNT THEM

         //scan only for /human#/shoulderCenter reference systems
         m_transformClientInt->getAllFrameIds(allFrameIds);
         filteredFrameIds.clear();

         for (int i = 0; i < allFrameIds.size(); i++)
         {
             //if (it2->compare(0, 6, "/human") == 0)
             if ((allFrameIds[i].compare(0, 6, "/human") == 0) && (allFrameIds[i].find("/shoulderCenter") != string::npos))
             {
                 filteredFrameIds.push_back(allFrameIds[i]);
             }
         }

         // calculate area of interest
         m_poi_x = 1.5;
         m_poi_y = 0;

         m_up_b = m_poi_y + m_area_radius;
         m_lower_b = m_poi_y - m_area_radius;
         m_left_b = m_poi_x - m_area_radius;
         m_rigth_b = m_poi_x + m_area_radius;

         n_humansT = 0;
         //get position of the humans
         if (filteredFrameIds.size() > 0)
         {
             int num_frame;
             for (auto it = filteredFrameIds.begin(); it != filteredFrameIds.end(); it++)
             {

                 yCDebug(CROWD_DETECTOR) << "FRAME: " << *it << ": ";

                 if (m_transformClientInt->getTransform(*it, targetFrame, transformMat) == false)
                 {

                     yCDebug(CROWD_DETECTOR) << "no transform between: " << *it << " and " << targetFrame;
                     continue;
                 }

                 //yCDebug(CROWD_DETECTOR) << transformMat.toString() ;

                 m_x_human = transformMat(0, 3);
                 m_y_human = transformMat(1, 3);

                 yCDebug(CROWD_DETECTOR) << "FRAME m_x_human: " << m_x_human;
                 yCDebug(CROWD_DETECTOR) << "FRAME m_y_human: " << m_y_human;

                 if (m_x_human < m_rigth_b && m_x_human > m_left_b && m_y_human > m_lower_b && m_y_human < m_up_b)
                 {
                     n_humansT++;
                     tstart = time(0);
                     yCDebug(CROWD_DETECTOR) << "+1 human in area ";
                 }
             }
         }
         yCDebug(CROWD_DETECTOR) << "CROWD DIMENSION: " << n_humansT;

        //persistence in time of a person
         tend = time(0);
         if (difftime(tend, tstart) > 3)
         {
             n_humans = 0;
         }
         else
         {
             if (n_humansT==0)
             {
                 n_humans = 1;
             }
             n_humans = n_humansT;
         }
         yCDebug(CROWD_DETECTOR) << "CROWD DIMENSION PERSISTENCE: " << n_humans;


        return true;
     }
     // Message handler. Just echo all received messages.
     bool MyModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
     {
         std::cout << "Got something, echo is on" << '\n';
         if (command.get(0).asString() == "quit")
             return false;
         else
             reply = command;
         return true;
     }
     // Configure function. Receive a previously initialized
     // resource finder object. Use it to configure your module.
     // If you are migrating from the old module, this is the function
     // equivalent to the "open" method.
     bool MyModule::configure(yarp::os::ResourceFinder &config)
     {
         count=0;
         if (!handlerPort.open("/myModule"))
             return false;

         // optional, attach a port to the module
         // so that messages received from the port are redirected
         // to the respond method
         attach(handlerPort);

         // configuring
         if (config.check("targetFrame"))
             targetFrame = config.find("targetFrame").asString();
         else
             targetFrame = "mobile_base_body_link";

         if (config.check("m_area_radius"))
             m_area_radius = config.find("m_area_radius").asFloat64();
         else
             m_area_radius = 0.5;

         if (config.check("name"))
             prefixName = config.find("name").asString();
         else
             prefixName = "crowdDetector";

         if (config.check("remoteTC"))
             remoteTCName = config.find("remoteTC").asString();
         else
             remoteTCName = "/transformServer";



         // connect to transform client and load interface
         Property pTC;
         pTC.put("device", "transformClient");
         pTC.put("local", "/" + prefixName + "/transformClient");
         pTC.put("remote", remoteTCName);
         pTC.put("period", "10");
         m_transformClientDriver.open(pTC);

         m_transformClientDriver.view(m_transformClientInt);
         if (m_transformClientInt == nullptr)
         {
             yCError(CROWD_DETECTOR) << "Unable to open Transform Client interface";
             return false;
         }
         yCInfo(CROWD_DETECTOR) << "tranformClient successfully open";

         tstart = time(0);

         return true;
     }
     // Interrupt function.
     bool MyModule::interruptModule()
     {
         std::cout << "Interrupting your module, for port cleanup" << '\n';
         return true;
     }
     // Close function, to perform cleanup.
     bool MyModule::close()
     {
         // optional, close port explicitly
         std::cout << "Calling close function\n";
         handlerPort.close();
         m_transformClientDriver.close();

         return true;
     }

 int main(int argc, char * argv[])
 {
     // initialize yarp network
     yarp::os::Network yarp;

     // create your module
     MyModule module;

     // prepare and configure the resource finder
     yarp::os::ResourceFinder rf;
     rf.configure(argc, argv);

     std::cout << "Configuring and starting module.\n";
     // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
     if (!module.runModule(rf)) {
         std::cerr << "Error module did not start\n";
     }

     std::cout << "Main returning..." << '\n';
     return 0;
 }
