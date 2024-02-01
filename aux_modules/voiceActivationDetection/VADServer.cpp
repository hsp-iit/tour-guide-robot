#include <iostream>

#include "VADServer.h"

VADServer::VADServer(std::shared_ptr<Detector> detector) : m_detector(detector) {}

bool VADServer::run_inference(const bool active) {
    m_detector->m_runInference = active;
    return true;
}