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
};

} // namespace pldm
