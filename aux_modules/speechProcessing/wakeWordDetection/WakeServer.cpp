#include <iostream>

#include "WakeServer.h"

WakeServer::WakeServer(std::shared_ptr<AudioCallback> audioCallback) : m_detector(audioCallback) {}

void WakeServer::stop() {
    m_detector->m_currentlyStreaming = false;
}