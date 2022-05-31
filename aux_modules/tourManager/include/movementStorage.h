
#ifndef BEHAVIOR_TOUR_ROBOT_MOVEMENT_STORAGE_H
#define BEHAVIOR_TOUR_ROBOT_MOVEMENT_STORAGE_H

#include <set>
#include <map>
#include <nlohmann/json.hpp>
#include <movement.h>
#include <dance.h>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

class MovementsContainer // Just for the purpose of easier serialization
{
private:
    std::set<std::string> m_partNames;
    std::map<std::string, Dance> m_dances;

public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(MovementsContainer, m_partNames, m_dances)

    /**
     * Invalid constructor
     */
    MovementsContainer() = default; // Returns an invalid MovementsContainer

    /**
     * Constructor
     *
     * @param partNames the names of all the support robot parts to move
     * @param dances the list of dances that are supported. A dance is a sequence of moves
     */
    MovementsContainer(std::set<std::string> partNames, std::map<std::string, Dance> dances);

    /**
     * Copy constructor.
     *
     * @param inputMove the data to copy.
     */
    MovementsContainer(const MovementsContainer &inputMove) = default;

    /**
     * @brief Move constructor.
     *
     * @param movingMove the MovementsContainer to be moved
     */
    MovementsContainer(MovementsContainer &&movingMove) noexcept = default;

    ~MovementsContainer() = default;

    /**
     * Copy operator=
     * @param m const reference to an MovementsContainer object
     * @return
     */
    MovementsContainer &operator=(const MovementsContainer &m) = default;

    [[nodiscard]] std::set<std::string>& GetPartNames();
    [[nodiscard]] std::map<std::string, Dance>& GetDances();
    [[nodiscard]] bool GetDance(const std::string &danceName, Dance &outDance) const;
};

class MovementStorage
{
private:
    MovementStorage() {}
    ~MovementStorage() {}

    MovementsContainer m_movementsContainer;

public:
    static MovementStorage &GetInstance(const std::string &pathJSONMovements);

    MovementStorage(const MovementStorage &) = delete;
    MovementStorage &operator=(const MovementStorage &) = delete;
    MovementStorage(MovementStorage &&) = delete;
    MovementStorage &operator=(MovementStorage &&) = delete;

    nlohmann::ordered_json ReadFileAsJSON(const std::string &path);
    bool LoadMovements(const std::string &pathJSONMovements);
    MovementsContainer &GetMovementsContainer();
};

#endif // BEHAVIOR_TOUR_ROBOT_MOVEMENT_STORAGE_H
