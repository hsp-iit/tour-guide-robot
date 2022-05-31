
#ifndef BEHAVIOR_TOUR_ROBOT_DANCE_H
#define BEHAVIOR_TOUR_ROBOT_DANCE_H

#include <vector>
#include <nlohmann/json.hpp>
#include <movement.h>
#include <yarp/os/LogStream.h>

using json = nlohmann::json;

class Dance
{
private:
    float m_duration;
    std::vector<Movement> m_movements;

public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Dance, m_movements)

    /**
     * Invalid constructor
     */
    Dance() = default; // Returns an invalid Dance

    /**
     * Constructor
     *
     * @param movements the movements of the dance
     */
    Dance(std::vector<Movement> movements);

    /**
     * Copy constructor.
     *
     * @param inputMove the data to copy.
     */
    Dance(const Dance &inputMove) = default;

    /**
     * @brief Move constructor.
     *
     * @param movingDance the Dance to be moved
     */
    Dance(Dance &&movingMove) noexcept = default;

    ~Dance() = default;

    /**
     * Copy operator=
     * @param m const reference to an Dance object
     * @return
     */
    Dance &operator=(const Dance &m) = default;

    /**
     * @return the total duration of the dance including queuing of same robot parts
     */
    [[nodiscard]] float GetDuration() const;

    /**
     * @return a copy of the vector of all the movement in this dance
     */
    [[nodiscard]] std::vector<Movement> GetMovements() const;

    /**
     * Updates the internal total duration of the dance object
     */
    void UpdateDuration();
};

#endif // BEHAVIOR_TOUR_ROBOT_DANCE_H
