#include <movement.h>

Movement::Movement(float time, float offset, std::string partName, std::vector<float> joints) : m_time(time),
                                                                                                m_offset(offset),
                                                                                                m_partName(partName),
                                                                                                m_joints(std::move(joints))
{
}

float Movement::GetTime() const
{
    return m_time;
}

float Movement::GetOffset() const
{
    return m_offset;
}

std::string Movement::GetPartName() const
{
    return m_partName;
}

std::vector<float> Movement::GetJoints() const
{
    return m_joints;
}
