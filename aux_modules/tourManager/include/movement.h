
#ifndef BEHAVIOR_TOUR_ROBOT_MOVEMENT_H
#define BEHAVIOR_TOUR_ROBOT_MOVEMENT_H

#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Movement
{
private:
    float m_time;
    float m_offset;
    std::string m_partName;
    std::vector<float> m_joints;

public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Movement, m_time, m_offset, m_partName, m_joints)

    /**
     * Invalid constructor
     */
    Movement() = default; // Returns an invalid Movement

    /**
     * Constructor
     *
     * @param time the time to execute the move
     * @param offset the offset of the joints
     * @param partName the name of the robot part in the ctpService
     * @param joints the joint position values
     */
    Movement(float time, float offset, std::string partName, std::vector<float> joints);

    /**
     * Copy constructor.
     *
     * @param inputMove the data to copy.
     */
    Movement(const Movement &inputMove) = default;

    /**
     * @brief Move constructor.
     *
     * @param movingMove the Movement to be moved
     */
    Movement(Movement &&movingMove) noexcept = default;

    ~Movement() = default;

    /**
     * Copy operator=
     * @param m const reference to an Movement object
     * @return
     */
    Movement &operator=(const Movement &m) = default;

    [[nodiscard]] float GetTime() const;
    [[nodiscard]] float GetOffset() const;
    [[nodiscard]] std::string GetPartName() const;
    [[nodiscard]] std::vector<float> GetJoints() const;
};

#endif // BEHAVIOR_TOUR_ROBOT_MOVEMENT_H
