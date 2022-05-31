#ifndef BEHAVIOR_TOUR_ROBOT_TOUR_MANAGER_H
#define BEHAVIOR_TOUR_ROBOT_TOUR_MANAGER_H

#include <tourStorage.h>
#include <movementStorage.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <googleDialog_IDL.h>
#include <googleSpeech_IDL.h>
#include <googleSynthesis_IDL.h>
#include <headSynchronizerRPC.h>
#include <tourManagerRPC.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <random>

class DialogflowCallback;
class TourManager : public yarp::os::RFModule, public tourManagerRPC
{
private:
    DialogflowCallback *m_dialogflowCallback;
    std::string m_name;
    double m_period;
    bool m_hasReachedPoI;
    bool m_isFirstStart;
    bool m_isSpeechInterrupted;
    int m_fallback_threshold;
    int m_fallback_repeat_counter;
    TourStorage *m_tourStorage;
    MovementStorage *m_moveStorage;
    PoI m_currentPoI;
    PoI m_genericPoI;
    int m_PoIndex;
    yarp::dev::Nav2D::Map2DLocation m_previousPoIloc;
    std::string m_previousPoIname;
    std::string m_last_valid_speak;
    std::mt19937 m_random_gen;
    std::default_random_engine m_rand_engine;
    std::uniform_int_distribution<std::mt19937::result_type> m_uniform_distrib;

    std::string m_pHeadSynchronizerName;
    std::string m_dialogName;
    std::string m_speechName;
    std::string m_synthesisName;
    std::string m_leftArmName;
    std::string m_rightArmName;
    std::string m_torsoName;
    std::string m_dialogflowOutputName;
    std::string m_dialogflowInputName;
    std::string m_tourManagerThriftPortName;
    std::string m_defaultLanguage;

    headSynchronizerRPC m_headSynchronizer;
    yarp::os::Port m_pHeadSynchronizer;
    yarp::os::Port m_tourManagerThriftPort;
    yarp::os::Port m_pSpeech;
    yarp::os::Port m_pSynthesis;
    yarp::os::Port m_pDialog;
    yarp::os::Port m_pDialogflowOutput;
    yarp::os::BufferedPort<yarp::os::Bottle> m_pDialogflowInput;
    std::map<std::string, yarp::os::Port &> m_pCtpService;
    googleSpeech_IDL m_speech;
    googleDialog_IDL m_Dialog;
    googleSynthesis_IDL m_Synthesis;
    yarp::dev::PolyDriver m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};

private:
    void BlockSpeak();
    void Speak(const std::string &text, bool isValid);
    float DoDance(const std::string &movement);
    void Signal(const std::string &param);
    bool SendMovement(float time, float offset, std::vector<float> joints, yarp::os::Port &port);
    void SendToDialogue(const std::string &command);
    bool NextPoI();
    bool UpdatePoI();

public:
    TourManager(const std::string &name, const std::string &pathJSONTours, const std::string &pathJSONMovements, const std::string &tourName);

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool interruptModule();
    virtual bool updateModule();

    bool InterpretCommand(const std::string &command);

    bool sendError(const std::string &error) override;
    bool recovered() override;
    bool isAtPoI() override;
    bool sendToPoI() override;
    std::string getCurrentPoIName();
};

class DialogflowCallback : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    DialogflowCallback(TourManager *tourManager);
    using yarp::os::TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle &b) override;

private:
    TourManager *m_tourManager;
};

#endif // BEHAVIOR_TOUR_ROBOT_TOUR_MANAGER_H