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
#include <filesystem>
#include <cstring>
#include "cper.hpp"

#undef DEBUG
#define OEM_EVENT               0xFA
#define SENSOR_TYPE_OEM         0xF0

#define CPER_FORMAT_TYPE        0

namespace pldm
{

static std::unique_ptr<sdbusplus::bus::match_t> pldmEventSignal;
static time_t prevTs = 0;
static int index = 0;

PldmMessagePollEvent::PldmMessagePollEvent(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    InstanceIdDb& instanceIdDb,
    pldm::requester::Handler<pldm::requester::Request>* handler) :
    EventHandlerInterface(eid, event, bus, instanceIdDb, handler)
{
    std::error_code ec;

    std::filesystem::create_directories(FAULT_LOG_PATH, ec);

    if (ec)
    {
        std::cerr << "Can not create fault log path \n" << std::endl;
    }
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

static void addSELLog(uint8_t TID, uint16_t eventID, AmpereSpecData *p)
{
    std::vector<uint8_t> evtData;
    std::string message = "PLDM RAS SEL Event";
    uint8_t recordType;
    uint8_t evtData1, evtData2, evtData3, evtData4, evtData5, evtData6;
    uint8_t socket;

    /*
     * OEM IPMI SEL Recode Format for RAS event:
     * evtData1:
     *    Bit [7:4]: eventClass
     *        0xF: oemEvent for RAS
     *    Bit [3:1]: Reserved
     *    Bit 0: SocketID
     *        0x0: Socket 0 0x1: Socket 1
     * evtData2:
     *    Event ID, indicates RAS PLDM sensor ID.
     * evtData3:
     *     Bit [7:4]: Payload Type
     *     Bit [3:0]: Error Type ID - Bit [11:8]
     * evtData4:
     *     Error Type ID - Bit [7:0]
     * evtData5:
     *     Error Sub Type ID high byte
     * evtData6:
     *     Error Sub Type ID low byte
     */
    socket = (TID == 1) ? 0 : 1;
    recordType = 0xD0;
    evtData1 = SENSOR_TYPE_OEM | socket;
    evtData2 = eventID;
    evtData3 = ((p->typeId.member.payloadType << 4) & 0xF0) |
               ((p->typeId.member.ipType >> 8) & 0xF);
    evtData4 = p->typeId.member.ipType ;
    evtData5 = p->subTypeId >> 8;
    evtData6 = p->subTypeId;
    /*
     * OEM data bytes
     *    Ampere IANA: 3 bytes [0x3a 0xcd 0x00]
     *    event data: 9 bytes [evtData1 evtData2 evtData3
     *                         evtData4 evtData5 evtData6
     *                         0x00     0x00     0x00 ]
     *    sel type: 1 byte [0xC0]
     */
    evtData.push_back(0x3a);
    evtData.push_back(0xcd);
    evtData.push_back(0);
    evtData.push_back(evtData1);
    evtData.push_back(evtData2);
    evtData.push_back(evtData3);
    evtData.push_back(evtData4);
    evtData.push_back(evtData5);
    evtData.push_back(evtData6);
    evtData.push_back(0);
    evtData.push_back(0);
    evtData.push_back(0);

    auto& bus = pldm::utils::DBusHandler::getBus();
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.Logging.IPMI", "/xyz/openbmc_project/Logging/IPMI",
            "xyz.openbmc_project.Logging.IPMI", "IpmiSelAddOem");
        method.append(message, evtData , recordType);

        auto selReply =bus.call(method);
        if (selReply.is_method_error())
        {
            std::cerr << "add SEL log error\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "call SEL log error: " << e.what() << "\n" ;
    }
}

static void addRedfishLog(std::string &primaryLogId, uint8_t type)
{
    auto& bus = pldm::utils::DBusHandler::getBus();
    std::map<std::string, std::variant<std::string, uint64_t>> params;

    switch (type) {
    case CPER_FORMAT_TYPE:
        params["Type"] = "CPER";
        break;
    //TBD: BERT, Diagnostic
    default:
        params["Type"] = "CPER";
        break;
    }
    params["PrimaryLogId"] = primaryLogId;
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.Dump.Manager", "/xyz/openbmc_project/dump/faultlog",
            "xyz.openbmc_project.Dump.Create", "CreateDump");
        method.append(params);

        auto response =bus.call(method);
        if (response.is_method_error())
        {
            std::cerr << "createDump error\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "call createDump error: " << e.what() << "\n" ;
    }

}

static std::string getUniqueEntryID(std::string &prefix)
{
    // Get the entry timestamp
    std::time_t curTs = time(0);
    std::tm timeStruct = *std::localtime(&curTs);
    char buf[80];
    // If the timestamp isn't unique, increment the index
    if (curTs == prevTs)
    {
        index++;
    }
    else
    {
        // Otherwise, reset it
        index = 0;
    }
    // Save the timestamp
    prevTs = curTs;
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &timeStruct);
    std::string uniqueId(buf);
    uniqueId = prefix + uniqueId;
    if (index > 0)
    {
        uniqueId += "_" + std::to_string(index);
    }
    return uniqueId;
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

    std::string prefix;
    switch (evtData.formatType) {
    case CPER_FORMAT_TYPE:
        prefix = "RAS_CPER_";
        break;
    default:
        prefix = "RAS_Unknown_";
        break;
    }

    std::string primaryLogId = getUniqueEntryID(prefix);
    std::string faultLogFilePath = std::string(FAULT_LOG_PATH) + primaryLogId;
    std::ofstream out (faultLogFilePath.c_str(), std::ofstream::binary);
    if(!out.is_open())
    {
        std::cerr << "Can not open ofstream for CPER binary file\n";
        return -1;
    }

    switch (evtData.formatType) {
    case CPER_FORMAT_TYPE:
        decodeCperRecord(data, pos, &ampHdr, out);
        break;
    //TBD: BERT, Diagnostic
    default:
        std::cerr << "Not support format type: " << evtData.formatType << "\n";
        break;
    }
    addSELLog(TID, eventID, &ampHdr);
    addRedfishLog(primaryLogId, evtData.formatType);

    out.close();
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
