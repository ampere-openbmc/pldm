#include "common/types.hpp"
#include "common/utils.hpp"

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>
#include <phosphor-logging/lg2.hpp>

#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <libpldm/pldm.h>
#include <algorithm>
#include <map>
#include <string_view>
#include <vector>
#include "event_manager.hpp"

namespace pldm
{

int EventManager::handleMessagePollEvent(const pldm_msg* request,
                                          size_t payloadLength,
                                          uint8_t /* formatVersion */,
                                          uint8_t tid,
                                          size_t eventDataOffset)
{
    lg2::info("received poll event tid={TID}", "TID", tid);

    uint8_t evtFormatVersion = 0;
    uint16_t evtId = 0;
    uint32_t evtDataTransferHandle = 0;
    auto eventData = reinterpret_cast<const uint8_t*>(request->payload) +
                     eventDataOffset;
    auto eventDataSize = payloadLength - eventDataOffset;

    auto rc = decode_pldm_message_poll_event_data(eventData, eventDataSize,
                                                  &evtFormatVersion, &evtId,
                                                  &evtDataTransferHandle);
    if (rc)
    {
        lg2::error("Failed to decode message poll event data, rc={RC}.", "RC",
                   rc);
        return rc;
    }

    if (devManager)
    {
        devManager->addEventMsg(tid, evtId, PLDM_MESSAGE_POLL_EVENT, 0);
    }
    return PLDM_SUCCESS;
}

int EventManager::handleSensorEvent(const pldm_msg* request,
                                          size_t payloadLength,
                                          uint8_t /* formatVersion */,
                                          uint8_t tid,
                                          size_t eventDataOffset)
{
    uint16_t sensorId = 0;
    uint8_t sensorEventClassType = 0;
    size_t eventClassDataOffset = 0;
    auto eventData = reinterpret_cast<const uint8_t*>(request->payload) +
                     eventDataOffset;
    auto eventDataSize = payloadLength - eventDataOffset;

    auto rc = decode_sensor_event_data(eventData, eventDataSize, &sensorId,
                                       &sensorEventClassType,
                                       &eventClassDataOffset);
    if (rc)
    {
        lg2::error("Failed to decode sensor event data, rc={RC}.", "RC",
                   rc);
        return rc;
    }
    switch (sensorEventClassType)
    {
        case PLDM_NUMERIC_SENSOR_STATE:
        {
            const uint8_t* sensorData = eventData + eventClassDataOffset;
            size_t sensorDataLength = eventDataSize - eventClassDataOffset;

            return processNumericSensorEvent(tid, sensorId, sensorData,
                                             sensorDataLength);
        }
        case PLDM_STATE_SENSOR_STATE:
        case PLDM_SENSOR_OP_STATE:
        default:
            lg2::info("unhandled sensor event, class type={CLASSTYPE}",
                      "CLASSTYPE", sensorEventClassType);
            return PLDM_ERROR;
    }
    return PLDM_SUCCESS;
}

int EventManager::processNumericSensorEvent(uint8_t tid, uint16_t sensorId,
                                            const uint8_t* sensorData,
                                            size_t sensorDataLength)
{
    uint8_t eventState = 0;
    uint8_t previousEventState = 0;
    uint8_t sensorDataSize = 0;
    uint32_t presentReading;
    auto rc = decode_numeric_sensor_data(sensorData, sensorDataLength,
                                         &eventState, &previousEventState,
                                         &sensorDataSize, &presentReading);
    if (rc)
    {
        return rc;
    }

    // RAS sensors
    if ((sensorId >= 191) && (sensorId <= 198))
    {
        if (devManager)
        {
            devManager->addEventMsg(tid, sensorId, PLDM_SENSOR_EVENT,
                             PLDM_NUMERIC_SENSOR_STATE);
        }
    }

    return PLDM_SUCCESS;
}

}
