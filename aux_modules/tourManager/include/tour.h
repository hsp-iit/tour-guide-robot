//
// Created by elandini on 1/17/2022.
//

#ifndef BEHAVIOR_TOUR_ROBOT_TOUR_H
#define BEHAVIOR_TOUR_ROBOT_TOUR_H

#include "poi.h"

class Tour
{
private:
    std::string m_currentLanguage;                                                         // The currently selected language
    std::unordered_map<std::string, std::unordered_map<std::string, PoI>> m_availablePoIs; // A map containing all the possible PoIs for
                                                                                           // this specific tour/location per language
    std::vector<std::string> m_activeTourPoIs;                                             // The list of the currently active PoIs (each element of this vector
                                                                                           // has to be a key of m_availablePoIs)

public:
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Tour, m_availablePoIs, m_activeTourPoIs)

    Tour() = default;

    /**
     * Constructor
     *
     * @param lang the language selected for the tour
     * @param availablePoIs a map containing the available PoIs for the Tour
     * NB: The PoIs in this container do not have to be only the active ones
     * @param activeTourPoIs an ordered std::vector containing the name of the PoIs that will be part of the actual tour
     */
    Tour(std::string lang,
         std::unordered_map<std::string, std::unordered_map<std::string, PoI>> availablePoIs, std::vector<std::string> activeTourPoIs);

    /**
     * Copy constructor.
     *
     * @param inputTour the data to copy.
     */
    Tour(const Tour &inputTour) = default;

    /**
     * @brief Move constructor.
     *
     * @param movingTour the Tour to be moved
     */
    Tour(Tour &&movingTour) noexcept = default;

    ~Tour() = default;

    /**
     * Copy operator=
     * @param t const reference to a Tour object
     * @return
     */
    Tour &operator=(const Tour &t) = default;

    /**
     * Used to get the list of the available languages
     * @return an std::vector containing the available languages
     */
    [[nodiscard]] std::vector<std::string> getAvailableLanguages() const;

    /**
     * Returns the tour currently set language
     * @return an std::string containing the Tour current language
     */
    [[nodiscard]] std::string getCurrentLanguage() const;

    /**
     * Tells whether or not a language is supported
     * @param lang the language to verify
     * @return true if lang is supported
     */
    bool languageSupported(const std::string &lang);

    /**
     * Sets the language for the tour. If the language is not available, the function will return false
     * @param langToSet the language to set
     * @return true if the selected language is available
     */
    bool setCurrentLanguage(const std::string &langToSet);

    /**
     * Used to get a specific PoI given its name
     * @param poiName the name of the desired PoI
     * @param outputPoI the PoI object to fill with the desired values
     * @return true if everything goes as expected
     */
    [[nodiscard]] bool getPoI(const std::string &poiName, PoI &outputPoI);

    /**
     * Used to get a specific PoI given its name and the language
     * @param poiName the name of the desired PoI
     * @param lang the language selected for the tour
     * @param outputPoI the PoI object to fill with the desired values
     * @return true if everything goes as expected
     */
    [[nodiscard]] bool getPoI(const std::string &poiName, const std::string &lang, PoI &outputPoI);

    /**
     * Used to get the list of active PoIs
     * @return an std::vector containing the active PoIs
     */
    [[nodiscard]] std::vector<std::string> getPoIsList() const;
};

#endif // BEHAVIOR_TOUR_ROBOT_TOUR_H
