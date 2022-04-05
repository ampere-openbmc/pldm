#pragma once

#include "requester/terminus_manager.hpp"

#include <sdbusplus/bus/match.hpp>

namespace pldm
{

class EventManager
{
  public:
    EventManager() = delete;
    EventManager(const EventManager&) = delete;
    EventManager(EventManager&&) = delete;
    EventManager& operator=(const EventManager&) = delete;
    EventManager& operator=(EventManager&&) = delete;
    ~EventManager() = default;

    explicit EventManager(terminus::Manager *dev):
        devManager(dev)
    {

    }

    int handleMessagePollEvent(const pldm_msg* request,
                                size_t payloadLength,
                                uint8_t /* formatVersion */, uint8_t tid,
                                size_t eventDataOffset);
    int handleSensorEvent(const pldm_msg* request,
                           size_t payloadLength,
                           uint8_t /* formatVersion */, uint8_t tid,
                           size_t eventDataOffset);

  protected:

    int processNumericSensorEvent(uint8_t tid, uint16_t sensorId,
                                  const uint8_t* sensorData,
                                  size_t sensorDataLength);

    void handleMCStateSensorEvent(uint8_t tid, [[maybe_unused]]uint16_t sensorId,
                uint32_t presentReading, [[maybe_unused]]uint8_t eventState);


    terminus::Manager *devManager;
};


}
