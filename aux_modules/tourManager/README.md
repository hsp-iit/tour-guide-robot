# Tour Manager Instructions

## JSON RULES

- The json tour file needs to have the following hierarchical structure:
    - Tour
        - Languages for tour
            - PoI's in the tour for that language
                - Commands that this PoI in this language can understand
                    - Actions that this command will take when called.

- There are three types of Actions: Speak, Signal, Move.
    - **Speak**: This action type makes the robot say the message that is in the params field of the Action. When the robot talks, the ears are automatically closed.
    - **Dance**: This action type makes the robot execute the dance specified by name in the params field of the Action. The dance name is defined in another json file. A dance is made of the names of x movements (by name) and the total duration of the dance is calculated by accounting for the queuing the ctpService will do for same parts.
    - **Signal**: This is a special action type, as it executes specific logic in the code. The logic to execute is defined in the params field of the Action. **Signals are blocking and can't be changed!** Valid signals are:
        - **startHearing**: Opens the ears of the robot. Usually used after we use a speak action to make the robot listen after a question.
        - **setLanguage**: Changes the language of the system. It modifies the language of the PoIs loaded in the tour as well as the synthesizer, dialogueflow etc. The signal is a prefix and the suffix of that signal defines the language and the voice e.x. setLanguage_en-US-Wavenet-C
        - **nextPoi**: Moves the index of the current PoI to the next or loops to the start if at the end, and updates the current PoI object from the Tour class.
        - **reset**: Resets the index of the current PoI to zero, and reloads the PoI object from the Tour class.
        - **delay_x**: Can be used to delay the execution of the command/action sequence at any place. The variable x is in seconds and can be a float.

- **Special considerations:**
    - **Fallback**: The special command "fallback" is used to notify the users that the robot did not understand what they said. Typically, the command fallback should have one action, speak. The speak actions just tells the user that the robot could not understand what it just hears. Normally, the next action should be to **open the ears** so that the user can retry with a different text, but the **fallback does that automatically**. Moreover, if the fallback has been triggered consecutively more than a predefined threshold, then it will **automatically repeat the last action of type speak from the last non-fallback command**. That promotes the splitting of single speak actions to two speaks actions: explanation, question. In this case, only the question part will be repeated after the predefined failed attempts to understand. 
    - **Multiples of commands**: A command can have multiple versions of itself. The format should add an index as a suffix to the variation. For example, you could have "greetings", "greetings1", "greetings2" etc. The default command should not have an index, and it is assumed to be 0. If there is more than one multiple of a command, the command to be executed is selected randomly from the variations using a uniform distribution.
    - **Blocking**: Every actions can be blocking or non blocking. In a list of actions inside a command, all actions are run in parallel (***in series with no delays***) if none of them is specified as blocking. The first one that is blocking, makes all be executed in "parallel" up to (including) the one that is blocking. It waits for **all** of the actions up to the blocking one to finish, and then proceeds to the others in the list with the same logic. For example if we have "b" for blocking and "n" for not blocking, then actions "n n b n b n n" will be executed first 3 in parallel waiting for all of them, then next 2 waiting, then last 2.

