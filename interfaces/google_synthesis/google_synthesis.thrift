# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
#
# googleSynthesis.thrift

/**
* googleSynthesis_IDL
*
* IDL Interface to google cloud speech processing.
*/

struct Bottle {
} (
  yarp.name = "yarp::os::Bottle"
  yarp.includefile = "yarp/os/Bottle.h"
)

service googleSynthesis_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();

    /**
     * Say the phrase.
     * @param string containing the phrase to synthesise
     * @return true/false on success/failure
     */
    bool say(1:string phrase);

    /**
     * Set the language of the synthesiser.
     * example: setLanguage en-US en-US-Wavenet-A
     * @param string containing the languageCode
     * @param string containing the voiceCode
     * @return true/false on success/failure
     */
    string setLanguage(1:string languageCode, 2:string voiceCode);

    /**
     * Set the pitch of the synthesiser.
     * @param double containing the desired pitch for the synthesiser
     * @return true/false on success/failure
     */
    bool setPitch(1:double pitch);

    /**
     * Set the speed of the synthesiser.
     * @param double containing the desired speed for the synthesiser
     * @return true/false on success/failure
     */
    bool setSpeed(1:double speed);

    /**
     * Get the language code of the synthesiser
     * @return string containing the language code
     */
    string getLanguageCode();

     /**
     * Get the voice code of the synthesiser
     * @return string containing the voice code
     */
    string getVoiceCode();

     /**
     * Get the pitch value of the synthesiser
     * @return string containing the pitch value
     */
    double getPitch();

    /**
     * Get the speed value of the synthesiser
     * @return string containing the speed code
     */
    double getSpeed();
}