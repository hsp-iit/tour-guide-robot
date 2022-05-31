/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

 /**
 * Enumerates the states every node can be in after execution during a particular
 * time step:
 * - "SKILL_SUCCESS" indicates that the node has completed running during this time step;
 * - "SKILL_FAILURE" indicates that the node has determined it will not be able to complete
 *   its task;
 * - "SKILL_RUNNING" indicates that the node has successfully moved forward during this
 *   time step, but the task is not yet complete;
 * - "SKILL_IDLE" indicates that the node is waiting to be (re)executed.
 */
enum SkillStatus {SKILL_IDLE, SKILL_RUNNING, SKILL_SUCCESS, SKILL_FAILURE}

service Skill_request {

    /**
     * get_status  Get the SkillStatus of the skill.
     *
     * return              The enum indicating the status of the skill.
     */
    SkillStatus get_status()

    /**
     * start  Starts skill.
     *
     * return               bool. True if the function ended correctly, false otherwise.
     */
    bool start();

    /**
     * stop  Send a stop request to the skill.
     *
     * return              void.
     */
     void stop();
}
