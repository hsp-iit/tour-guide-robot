#include <movementStorage.h>

YARP_LOG_COMPONENT(MOVEMENT_STORAGE, "behavior_tour_robot.aux_modules.movementstorage", yarp::os::Log::TraceType)

/**
 *
 * START OF MOVEMENT_STORAGE
 *
 */

MovementStorage &MovementStorage::GetInstance(const std::string &pathJSONMovements)
{
    static MovementStorage instance; // Guaranteed to be destroyed. Instantiated on first use.
    instance.LoadMovements(pathJSONMovements);
    return instance;
}

bool MovementStorage::LoadMovements(const std::string &pathJSONMovements)
{
    // Load movements
    nlohmann::ordered_json movements_json = ReadFileAsJSON(pathJSONMovements);
    MovementsContainer loadedMovements = movements_json.get<MovementsContainer>();
    m_movementsContainer = loadedMovements;
    yCInfo(MOVEMENT_STORAGE) << "Loaded:" << m_movementsContainer.GetPartNames().size() << "robot parts.";
    yCInfo(MOVEMENT_STORAGE) << "Loaded:" << m_movementsContainer.GetDances().size() << "dances.";

    for (auto &dance : m_movementsContainer.GetDances()) // Needs to be done here as json parser calls the default contructor and copies variables
    {
        dance.second.UpdateDuration();
    }

    return true;
}

nlohmann::ordered_json MovementStorage::ReadFileAsJSON(const std::string &path)
{
    std::ifstream file(path);
    std::string sentence = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return nlohmann::ordered_json::parse(sentence);
}

MovementsContainer &MovementStorage::GetMovementsContainer()
{
    return m_movementsContainer;
}

/**
 *
 * END OF MOVEMENT_STORAGE
 *
 */

/**
 *
 * START OF MOVEMENT_CONTAINER
 *
 */

MovementsContainer::MovementsContainer(std::set<std::string> partNames, std::map<std::string, Dance> dances) : m_partNames(std::move(partNames)),
                                                                                                               m_dances(std::move(dances))
{
}

std::set<std::string> &MovementsContainer::GetPartNames()
{
    return m_partNames;
}

std::map<std::string, Dance> &MovementsContainer::GetDances()
{
    return m_dances;
}

bool MovementsContainer::GetDance(const std::string &danceName, Dance &outDance) const
{
    auto foundDance = m_dances.find(danceName);
    if (foundDance != m_dances.end())
    {
        outDance = foundDance->second;
        return true;
    }
    return false;
}

/**
 *
 * END OF MOVEMENT_CONTAINER
 *
 */