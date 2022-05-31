// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#ifndef BEHAVIOR_TOUR_ROBOT_FILEWRITER_H
#define BEHAVIOR_TOUR_ROBOT_FILEWRITER_H

#include <string>
#include <mutex>
#include <fstream>

class FileWriter
{
public:
    FileWriter() = default;
    bool writeToFile(const std::string &question,
                     const std::string &answer,
                     const std::string &poiName);
    bool close();
    bool configure(const std::string &filename);

private:
    std::ofstream fileToWrite;
    std::mutex m_mutex; /** Internal mutex. **/
};

#endif // BEHAVIOR_TOUR_ROBOT_FILEWRITER_H
