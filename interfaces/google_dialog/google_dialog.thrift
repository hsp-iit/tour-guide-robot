# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
#
# googleDialog.thrift

/**
* googleDialog_IDL
*
* IDL Interface to google cloud speech processing.
*/

struct Bottle {
} (
  yarp.name = "yarp::os::Bottle"
  yarp.includefile = "yarp/os/Bottle.h"
)

service googleDialog_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();

     /**
     * Get the state of the google service.
     * @return a string containing the state
     */
    string getState();

     /**
     * Get the time needed for the google service to process the request.
     * @return a double containing the processing time
     */
    i64 getProcessingTime();

     /**
     * Reset the conversation.
     */
    bool resetDialog();

    /**
     * Set the language of the synthesiser.
     * example: setLanguage en-US
     * @param string containing the languageCode
     * @return true/false on success/failure
     */
    bool setLanguage(1:string languageCode);

    /**
     * Get the language code of the synthesiser
     * @return string containing the language code
     */
    string getLanguageCode();

}