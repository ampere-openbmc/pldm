#include "config.h"

#include "pldm_message_poll_event.hpp"

#include "libpldm/pldm.h"

#include <assert.h>
#include <systemd/sd-journal.h>

#include <nlohmann/json.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>

#undef DEBUG
#define OEM_EVENT 0xFA

namespace pldm
{

static std::unique_ptr<sdbusplus::bus::match_t> pldmEventSignal;
static std::map<uint16_t, int> priorityMap;

PldmMessagePollEvent::PldmMessagePollEvent(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm::dbus_api::Requester& requester,
    pldm::requester::Handler<pldm::requester::Request>* handler) :
    EventHandlerInterface(eid, event, bus, requester, handler)
{
    // register event class handler
    registerEventHandler(PLDM_MESSAGE_POLL_EVENT,
                         [&](uint8_t TID, uint8_t eventClass, uint16_t eventID,
                             const std::vector<uint8_t> data) {
                             return pldmPollForEventMessage(TID, eventClass,
                                                            eventID, data);
                         });
    // TODO: update OEM class handler
    registerEventHandler(OEM_EVENT, [&](uint8_t TID, uint8_t eventClass,
                                        uint16_t eventID,
                                        const std::vector<uint8_t> data) {
        return pldmPollForEventMessage(TID, eventClass, eventID, data);
    });

    handlePldmDbusEventSignal();
}

int PldmMessagePollEvent::pldmPollForEventMessage(uint8_t TID,
                                                  uint8_t eventClass,
                                                  uint16_t eventID,
                                                  std::vector<uint8_t> data)
{
#ifdef DEBUG
    std::cout << "\nOUTPUT DATA\n";
    for (auto it = data.begin(); it != data.end(); it++)
    {
        std::cout << std::hex << (unsigned)*it << " ";
    }
    std::cout << "\n";
#endif

    int priority = msgPriority;
    auto it = priorityMap.find(eventID);
    if (it != priorityMap.end())
    {
        priority = priorityMap.at(eventID);
    }

    // add event message to journal log
    addJournalRecord("SYSTEM_ERROR_EVENT:" + std::to_string(eventID), TID,
                     eventClass, eventID, priority, data);

    if (it != priorityMap.end())
    {
        priorityMap.erase(it);
    }

    return data.size();
}

void PldmMessagePollEvent::handlePldmDbusEventSignal()
{
    pldmEventSignal = std::make_unique<sdbusplus::bus::match_t>(
        pldm::utils::DBusHandler::getBus(),
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("PldmMessagePollEvent") +
            sdbusplus::bus::match::rules::path("/xyz/openbmc_project/pldm") +
            sdbusplus::bus::match::rules::interface(
                "xyz.openbmc_project.PLDM.Event"),
        [&](sdbusplus::message::message& msg) {
            try
            {
                uint8_t msgTID{};
                uint8_t msgEventClass{};
                uint8_t msgFormatVersion{};
                uint16_t msgEventID{};
                uint32_t msgEventDataTransferHandle{};

                // Read the msg and populate each variable
                msg.read(msgTID, msgEventClass, msgFormatVersion, msgEventID,
                         msgEventDataTransferHandle);
#ifdef DEBUG
                std::cout << "\n->Coming DBUS Event Signal\n"
                          << "TID: " << std::hex << (unsigned)msgTID << "\n"
                          << "msgEventClass: " << std::hex
                          << (unsigned)msgEventClass << "\n"
                          << "msgFormatVersion: " << std::hex
                          << (unsigned)msgFormatVersion << "\n"
                          << "msgEventID: " << std::hex << msgEventID << "\n"
                          << "msgEventDataTransferHandle: " << std::hex
                          << msgEventDataTransferHandle << "\n";
#endif
                try
                {
                    // add the priority
                    priorityMap[msgEventID] = 2;
                    this->enqueueCriticalEvent(msgEventID);
                }
                catch (const std::exception& e)
                {
                    std::cerr << "subscribePldmDbusEventSignal failed\n"
                              << e.what() << std::endl;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "subscribePldmDbusEventSignal failed\n"
                          << e.what() << std::endl;
            }
        });
}

} // namespace pldm
