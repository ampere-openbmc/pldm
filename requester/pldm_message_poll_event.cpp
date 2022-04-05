#include "config.h"

#include "pldm_message_poll_event.hpp"

#include "libpldm/pldm.h"
#include "common/utils.hpp"

#include <assert.h>
#include <systemd/sd-journal.h>

#include <nlohmann/json.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>
#ifdef AMPERE
#include "com/ampere/CrashCapture/Trigger/server.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>
#include <filesystem>
#include <cstring>
#include "cper.hpp"

#undef DEBUG
#define OEM_EVENT               0xFA

#define CPER_FORMAT_TYPE        0

using namespace pldm::utils;

namespace pldm
{

PldmMessagePollEvent::PldmMessagePollEvent(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    InstanceIdDb& instanceIdDb,
    pldm::requester::Handler<pldm::requester::Request>* handler) :
    EventHandlerInterface(eid, event, bus, instanceIdDb, handler)
{
    if (!std::filesystem::is_directory(CPER_LOG_PATH))
         std::filesystem::create_directories(CPER_LOG_PATH);

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
}

int PldmMessagePollEvent::pldmPollForEventMessage(uint8_t TID,
                                                  uint8_t /*eventClass*/,
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
    long pos;
    AmpereSpecData ampHdr;
    CommonEventData evtData;
    pos = 0;

    std::memcpy((void*)&evtData, (void*)&data[pos], sizeof(CommonEventData));
    pos += sizeof(CommonEventData);

    if (!std::filesystem::is_directory(CPER_LOG_DIR))
         std::filesystem::create_directories(CPER_LOG_DIR);

    std::string cperFile = std::string(CPER_LOG_DIR) + "cper.dump." + std::to_string(TID);
    std::ofstream out (cperFile.c_str(), std::ofstream::binary);
    if(!out.is_open())
    {
        std::cerr << "Can not open ofstream for CPER binary file\n";
        return -1;
    }
    decodeCperRecord(data, pos, &ampHdr, out);
    out.close();

    std::string prefix = "RAS_CPER_";
    std::string primaryLogId = pldm::utils::getUniqueEntryID(prefix);
    std::string type = "CPER";
    std::string faultLogFilePath = std::string(CPER_LOG_PATH) + primaryLogId;
    std::filesystem::copy(cperFile.c_str(), faultLogFilePath.c_str());
    std::filesystem::remove(cperFile.c_str());

    addCperSELLog(TID, eventID, &ampHdr);
    pldm::utils::addFaultLogToRedfish(primaryLogId, type);

#ifdef AMPERE
    if (ampHdr.typeId.member.isBert)
    {
        constexpr auto rasSrv = "com.ampere.CrashCapture.Trigger";
        constexpr auto rasPath = "/com/ampere/crashcapture/trigger";
        constexpr auto rasIntf = "com.ampere.CrashCapture.Trigger";
        using namespace sdbusplus::xyz::openbmc_project::Logging::server;
        std::variant<std::string> value("com.ampere.CrashCapture.Trigger.TriggerAction.Bert");
        try
        {
            auto& bus = pldm::utils::DBusHandler::getBus();
            auto method = bus.new_method_call(rasSrv,rasPath,
                dbusProperties, "Set");
            method.append(rasIntf, "TriggerActions", value);
            bus.call_noreply(method);
        }
        catch(const std::exception& e)
        {
            std::cerr << "call BERT trigger error: " << e.what() << "\n" ;
        }
    }
#endif

    return data.size();
}


} // namespace pldm
