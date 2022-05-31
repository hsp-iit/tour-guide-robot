
#ifndef BEHAVIOR_TOUR_ROBOT_TOUR_STORAGE_H
#define BEHAVIOR_TOUR_ROBOT_TOUR_STORAGE_H

#include "poi.h"
#include "tour.h"
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <nlohmann/json.hpp>

class TourStorage
{
private:
    TourStorage() {}
    ~TourStorage() {}

    Tour m_loadedTour; // The Tour object that was loaded
public:
    static TourStorage &GetInstance(const std::string &pathJSONTours, const std::string &tourName);

    TourStorage(const TourStorage &) = delete;
    TourStorage &operator=(const TourStorage &) = delete;
    TourStorage(TourStorage &&) = delete;
    TourStorage &operator=(TourStorage &&) = delete;

    nlohmann::ordered_json ReadFileAsJSON(const std::string &path);
    bool WriteJSONtoFile(const nlohmann::ordered_json &j, const std::string &path);
    bool LoadTour(const std::string &pathTours, const std::string &tourName);
    Tour &GetTour();
};

#endif // BEHAVIOR_TOUR_ROBOT_TOUR_STORAGE_H
