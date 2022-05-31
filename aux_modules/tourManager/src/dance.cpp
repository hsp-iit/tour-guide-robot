#include <dance.h>

YARP_LOG_COMPONENT(DANCE, "behavior_tour_robot.aux_modules.TourManager.Dance", yarp::os::Log::TraceType)

Dance::Dance(std::vector<Movement> movements) : m_movements(std::move(movements))
{
}

void Dance::UpdateDuration()
{
    std::map<std::string, float> durationPerPart;

    for (Movement movement : m_movements)
    { // For each of the movements in the dance
        float moveTime = movement.GetTime();
        std::string partName = movement.GetPartName();

        if (moveTime != 0.0f)
        {
            if (durationPerPart.find(partName) == durationPerPart.end())
            { // If the part name for the dance is seen for the first time
                durationPerPart.insert({partName, moveTime});
                continue;
            }
            durationPerPart.at(partName) += moveTime; // If the part name has already been seen, add to the total time and expect queing
        }
    }

    float longestPartMoveTime = 0.0f;

    for (auto part : durationPerPart)
    {
        if (part.second > longestPartMoveTime)
        {
            longestPartMoveTime = part.second;
        }
    }
    m_duration = longestPartMoveTime;
}

float Dance::GetDuration() const
{
    return m_duration;
}

std::vector<Movement> Dance::GetMovements() const
{
    return m_movements;
}
