// SPDX-FileCopyrightText: 2022 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause

#include "FileWriter.h"

bool FileWriter::configure(const std::string &filename)
{
    std::time_t time_epoch = std::time(0);
    fileToWrite.open(filename + "_" + std::to_string(time_epoch) + ".csv");
    if (!fileToWrite.is_open())
    {
        return false;
    }
    fileToWrite << "Question,Answer,PoI_Name" << std::endl;
    return true;
}

bool FileWriter::writeToFile(const std::string &question,
                             const std::string &answer,
                             const std::string &poiName)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    fileToWrite << question << "," << answer << "," << poiName << std::endl;
    return true;
}

bool FileWriter::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    fileToWrite.close();
    return true;
}
