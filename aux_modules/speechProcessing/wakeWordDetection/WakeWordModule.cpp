#include "WakeWordModule.h" 

#include <iostream>

YARP_LOG_COMPONENT(WAKEWORDMODULE, "tour_guide_robot.speech.wakeWordDetection.WakeWordModule", yarp::os::Log::TraceType);

bool WakeWordModule::configure(yarp::os::ResourceFinder &rf) {
    std::string audioPortOutName = rf.check("filtered_audio_output_port_name",
                                                    yarp::os::Value("/wake/audio:o"),
                                                    "The name of the output port.")
                                               .asString();

    std::string audioPortInName = rf.check("audio_input_port_name", yarp::os::Value("/wake/audio:i"),
                                       "The name of the input port for the audio.")
                                  .asString();;

    std::string wakeWordServerPort = rf.check("vad_server_port_name", yarp::os::Value("/wake/rpc:i"),
                                                  "Name of the input port to stop streaming")
                                             .asString();

    std::string accessKey = rf.check("accessKey", yarp::os::Value("E3HSLWAlzc76SFsflAy+9NSJotzp4u1VQIKU63sdiyc9CzqQL8HRDg=="),
                                                  "Porcupine access key")
                                             .asString();

    std::string modelPath = rf.check("model_path", yarp::os::Value("/home/user1/share/porcupine/lib/common/porcupine_params.pv"),
                                                  "Path to wake word detector model")
                                             .asString();

    std::string keywordPath = rf.check("keyword_path", yarp::os::Value("/home/user1/share/porcupine_playground/hey-r-one_en_linux_v3_0_0.ppn"),
                                                  "Path to ppn file containing keyword info")
                                             .asString();

    float detectorSensitivity = rf.check("detector_sensitivity", yarp::os::Value(0.6),
                                                  "Sensivity of wake word detector, higher will mean more false positves less false negatives")
                                             .asFloat32();

    m_callback = std::make_shared<AudioCallback>(audioPortOutName, accessKey, modelPath, keywordPath, detectorSensitivity);

    if (!m_audioPortIn.open(audioPortInName))
    {
        yCDebug(WAKEWORDMODULE) << "Cannot open port " << audioPortInName;
        return false;
    } 
    m_audioPortIn.useCallback(*m_callback);

    if (!m_rpcPort.open(wakeWordServerPort))
    {
        yCDebug(WAKEWORDMODULE) << "Cannot open port " << wakeWordServerPort;
        return false;
    } 
    m_rpcServer = std::make_unique<WakeServer>(m_callback);

    m_rpcServer->yarp().attachAsServer(m_rpcPort);

    return true;
}

bool WakeWordModule::close()
{
    m_audioPortIn.close();
    return true;
}

bool WakeWordModule::updateModule()
{
    return true;
}