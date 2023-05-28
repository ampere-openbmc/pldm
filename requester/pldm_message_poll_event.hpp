#pragma once

#include "libpldm/base.h"
#include "libpldm/fru.h"
#include "libpldm/platform.h"

#include "common/types.hpp"
#include "common/utils.hpp"
#include "event_hander_interface.hpp"
#include "libpldmresponder/event_parser.hpp"
#include "requester/handler.hpp"

#include <systemd/sd-journal.h>

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>
#include <sdeventplus/utility/timer.hpp>

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace pldm
{

class PldmMessagePollEvent : public EventHandlerInterface
{

  public:
    ~PldmMessagePollEvent() = default;
    PldmMessagePollEvent() = delete;
    PldmMessagePollEvent(const PldmMessagePollEvent&) = delete;
    PldmMessagePollEvent(PldmMessagePollEvent&&) = default;
    PldmMessagePollEvent& operator=(const PldmMessagePollEvent&) = delete;
    PldmMessagePollEvent& operator=(PldmMessagePollEvent&&) = default;

    explicit PldmMessagePollEvent(
        uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
        InstanceIdDb& instanceIdDb,
        pldm::requester::Handler<pldm::requester::Request>* handler);

  private:
    int pldmPollForEventMessage(uint8_t TID, uint8_t eventClass,
                                uint16_t eventID, std::vector<uint8_t> data);

    void handlePldmDbusEventSignal();

    int msgPriority = 5; // notice

    void toHexStr(const std::vector<uint8_t>& data, std::string& hexStr)
    {
        std::stringstream stream;
        stream << std::hex << std::uppercase << std::setfill('0');
        for (int v : data)
        {
            stream << std::setw(2) << v;
        }
        hexStr = stream.str();
    };

    template <typename... T>
    void addJournalRecord([[maybe_unused]] const std::string& message,
                          std::uint8_t tid, std::uint8_t eventClass,
                          std::uint16_t eventId, int priority,
                          const std::vector<uint8_t>& eventData,
                          [[maybe_unused]] T&&... metadata)
    {
        try
        {
            std::string eventDataStr;
            toHexStr(eventData, eventDataStr);
            sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i",
                            priority, "TID=%d", tid, "EVENT_CLASS=%d",
                            eventClass, "EVENT_ID=%ld", eventId,
                            "EVENT_DATA=%s", eventDataStr.c_str(),
                            std::forward<T>(metadata)..., NULL);
        }
        catch (const std::exception& e)
        {
            std::cerr << "ERROR:\n" << e.what() << std::endl;
        }
    }
};

} // namespace pldm
