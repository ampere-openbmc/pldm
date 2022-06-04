#include "config.h"

#include "terminus_handler.hpp"

#include <sdeventplus/source/time.hpp>

#include <chrono>

namespace pldm
{

namespace terminus
{

using namespace pldm::sensor;
using EpochTimeUS = uint64_t;
std::string fruPath = "/xyz/openbmc_project/pldm/fru";

TerminusHandler::TerminusHandler(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm_pdr* repo, pldm_entity_association_tree* entityTree,
    pldm_entity_association_tree* bmcEntityTree,
    pldm::requester::Handler<pldm::requester::Request>* handler,
    pldm::InstanceIdDb& instanceIdDb) :
    eid(eid),
    bus(bus), event(event), repo(repo), entityTree(entityTree),
    bmcEntityTree(bmcEntityTree), handler(handler),
    instanceIdDb(instanceIdDb), _state()
{}

TerminusHandler::~TerminusHandler()
{
    this->frus.clear();
    this->compNumSensorPDRs.clear();
    this->effecterAuxNamePDRs.clear();
    this->effecterPDRs.clear();
    this->_state.clear();
    this->_sensorObjects.clear();
    this->_effecterLists.clear();
}

requester::Coroutine TerminusHandler::discoveryTerminus()
{
    std::cerr << "Discovery Terminus: " << unsigned(eid) << std::endl;
    /*
     * 1. Initialize PLDM if PLDM Type is supported
     *
     * 1.1 Get supported PLDM Types
     *
     * 1.2. If PLDM for BIOS control and configuration is supported
     * 1.2.1 Set the date and time using the SetDateTime command
     *
     * 1.3. If PLDM Base Type is supported, get PLDM Base commands
     * 1.3.1 Get TID
     * 1.3.2 Get PLDM Commands
     *
     * 1.4. If FRU Type is supported, issue these FRU commands
     * 1.4.1 Get FRU Meta data via GetFRURecordTableMetadata
     * 1.4.2 Get FRU Table data via GetFRURecordTable
     *
     * 1.5. If PLDM Platform Type is supported, get PLDM Platform commands
     * 1.5.1 Prepare to receive event notification SetEventReceiver
     * 1.5.2 Get all Sensor/Effecter/Association information via GetPDR command
     *
     */
    auto rc = co_await getPLDMTypes();
    if (rc)
    {
        std::cerr << "Failed to getPLDMTypes, rc=" << unsigned(rc) << std::endl;
        co_return rc;
    }
    /* Received the response, terminus is on */
    this->responseReceived = true;

    if (supportPLDMType(PLDM_BASE))
    {
        rc = co_await getPLDMCommands();
        if (rc)
        {
            std::cerr << "Failed to getPLDMCommands, rc=" << unsigned(rc)
                      << std::endl;
        }
    }

    if (supportPLDMType(PLDM_BASE))
    {
        rc = co_await getTID();
        if (rc)
        {
            std::cerr << "Failed to getTID, rc=" << unsigned(rc) << std::endl;
        }
    }

    if (supportPLDMType(PLDM_BIOS))
    {
        rc = co_await setDateTime();
        if (rc)
        {
            std::cerr << "Failed to setDateTime, rc=" << unsigned(rc)
                      << std::endl;
        }
    }

    uint16_t totalTableRecords = 0;
    if (supportPLDMType(PLDM_FRU))
    {
        rc = co_await getFRURecordTableMetadata(&totalTableRecords);
        if (rc)
        {
            std::cerr << "Failed to getFRURecordTableMetadata, "
                      << "rc=" << unsigned(rc) << std::endl;
        }
        if (!totalTableRecords)
        {
            std::cerr << "Number of record table is not correct." << std::endl;
        }
    }

    if ((totalTableRecords != 0) && supportPLDMType(PLDM_FRU))
    {
        rc = co_await getFRURecordTable(totalTableRecords);
        if (rc)
        {
            std::cerr << "Failed to getFRURecordTable, "
                      << "rc=" << unsigned(rc) << std::endl;
        }
    }

    if (supportPLDMType(PLDM_PLATFORM))
    {
        rc = co_await setEventReceiver();
        if (rc)
        {
            std::cerr << "Failed to setEventReceiver, rc=" << unsigned(rc)
                      << std::endl;
        }
    }

    if (supportPLDMType(PLDM_PLATFORM))
    {
        if (debugGetPDR)
        {
            startTime = std::chrono::system_clock::now();
            std::cerr << eidToName.second << " Start GetPDR at "
                      << getCurrentSystemTime() << std::endl;
        }

        rc = co_await getDevPDR(0);
        if (rc)
        {
            std::cerr << "Failed to setEventReceiver, rc=" << unsigned(rc)
                      << std::endl;
        }
        else
        {
            readCount = 0;
            if (debugGetPDR)
            {
                std::chrono::duration<double> elapsed_seconds =
                    std::chrono::system_clock::now() - startTime;
                std::cerr << eidToName.second << " Finish get all PDR "
                          << elapsed_seconds.count() << "s at "
                          << getCurrentSystemTime() << std::endl;
            }
            if (this->compNumSensorPDRs.size() > 0)
            {
                this->createCompactNummericSensorIntf(this->compNumSensorPDRs);
            }
            if (this->effecterAuxNamePDRs.size() > 0)
            {
                this->parseAuxNamePDRs(this->effecterAuxNamePDRs);
            }
            if (this->effecterPDRs.size() > 0)
            {
                this->createNummericEffecterDBusIntf(this->effecterPDRs);
            }
            if (_state.size() > 0)
            {
                createdDbusObject = true;
            }
        }
    }

    co_return PLDM_SUCCESS;
}

bool TerminusHandler::supportPLDMType(const uint8_t type)
{
    if (devInfo.supportedTypes[type / 8].byte & (1 << (type % 8)))
    {
        return true;
    }

    return false;
}

bool TerminusHandler::supportPLDMCommand(const uint8_t type,
                                         const uint8_t command)
{
    if (!supportPLDMType(type))
    {
        return false;
    }
    if (devInfo.supportedCmds[type].cmdTypes[command / 8].byte &
        (1 << (command % 8)))
    {
        return true;
    }

    return false;
}

std::string TerminusHandler::getCurrentSystemTime()
{
    auto currentTime = std::chrono::system_clock::now();
    char buffer[80];
    char buffer1[4];

    auto transformed = currentTime.time_since_epoch().count() / 1000000;
    auto millis = transformed % 1000;

    std::time_t tt;
    tt = std::chrono::system_clock::to_time_t(currentTime);
    auto timeinfo = localtime(&tt);
    strftime(buffer, 80, "%F %H:%M:%S", timeinfo);
    snprintf(buffer1, 4, "%03d", (int)millis);
    return std::string(buffer) + ":" + std::string(buffer1);
}

requester::Coroutine TerminusHandler::getPLDMTypes()
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " get the PLDM Types." << std::endl;

    std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) +
                                    PLDM_GET_TYPES_RESP_BYTES);
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto instanceId = requester.getInstanceId(eid);

    auto rc = encode_get_types_req(instanceId, request);
    if (rc != PLDM_SUCCESS)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "Failed to encode_get_types_req, rc = " << unsigned(rc)
                  << std::endl;
        co_return rc;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BASE)
                  << ", cmd= " << unsigned(PLDM_GET_PLDM_TYPES)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << std::endl;
        co_return rc;
    }

    std::vector<bitfield8_t> types(8);
    rc = decode_get_types_resp(response, respMsgLen, &cc, types.data());
    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr << "Faile to decode_get_types_resp, Message Error: "
                  << "rc=" << unsigned(rc) << ",cc=" << unsigned(cc)
                  << std::endl;
        for (int i = 0; i < 8; i++)
        {
            devInfo.supportedTypes[i].byte = 0;
        }
        co_return rc;
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            devInfo.supportedTypes[i] = types[i];
        }
    }

    co_return cc;
}

requester::Coroutine TerminusHandler::getPLDMCommands()
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " get the supported PLDM Types." << std::endl;

    uint8_t type = PLDM_BASE;
    while ((type < PLDM_MAX_TYPES) && supportPLDMType(type))
    {
        auto rc = co_await getPLDMCommand(type);
        if (rc)
        {
            std::cerr << "Failed to getPLDMCommand, Type=" << unsigned(type)
                      << " rc =" << unsigned(rc) << std::endl;
        }
        type++;
    }
    co_return PLDM_SUCCESS;
}

requester::Coroutine TerminusHandler::getPLDMCommand(const uint8_t& pldmTypeIdx)
{
    auto instanceId = requester.getInstanceId(eid);
    Request requestMsg(sizeof(pldm_msg_hdr) + PLDM_GET_COMMANDS_REQ_BYTES);
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    ver32_t version{0xFF, 0xFF, 0xFF, 0xFF};
    auto rc =
        encode_get_commands_req(instanceId, pldmTypeIdx, version, request);
    if (rc != PLDM_SUCCESS)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "Failed to encode_get_commands_req, rc = " << unsigned(rc)
                  << std::endl;
        co_return rc;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BASE)
                  << ", cmd= " << unsigned(PLDM_GET_PLDM_COMMANDS)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    /* Process response */
    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BASE)
                  << ", cmd= " << unsigned(PLDM_GET_PLDM_COMMANDS)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    std::vector<bitfield8_t> cmdTypes(32);
    rc = decode_get_commands_resp(response, respMsgLen, &cc, cmdTypes.data());
    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr << "Response Message Error: "
                  << "rc=" << unsigned(rc) << ",cc=" << unsigned(cc)
                  << std::endl;
        for (auto i = 0; i < 32; i++)
        {
            devInfo.supportedCmds[pldmTypeIdx].cmdTypes[i].byte = 0;
        }
        co_return rc;
    }

    uint8_t i = 0;
    for (const auto& cmd : cmdTypes)
    {
        devInfo.supportedCmds[pldmTypeIdx].cmdTypes[i++].byte = cmd.byte;
    }

    co_return cc;
}

requester::Coroutine TerminusHandler::getTID()
{
    std::cerr << "Discovery Terminus: " << unsigned(eid) << " get TID."
              << std::endl;
    auto instanceId = requester.getInstanceId(eid);
    Request requestMsg(sizeof(pldm_msg_hdr));
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto rc = encode_get_tid_req(instanceId, request);
    if (rc)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "encode_get_tid_req failed. rc=" << unsigned(rc)
                  << std::endl;
        ;
        co_return rc;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BASE)
                  << ", cmd= " << unsigned(PLDM_GET_TID)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BASE)
                  << ", cmd= " << unsigned(PLDM_GET_TID)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t tid = PLDM_TID_RESERVED;
    rc = decode_get_tid_resp(response, respMsgLen, &cc, &tid);
    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr << "Faile to decode_get_tid_resp, Message Error: "
                  << "rc=" << unsigned(rc) << ",cc=" << unsigned(cc)
                  << std::endl;
        devInfo.tid = 0xFF;
        co_return cc;
    }

    devInfo.tid = tid;
    std::cerr << "Discovery Terminus: EID=" << unsigned(eid) << " TID="
              << unsigned(tid) << std::endl;

    co_return cc;
}

requester::Coroutine TerminusHandler::setEventReceiver()
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " get set Event Receiver." << std::endl;
    uint8_t eventMessageGlobalEnable =
        PLDM_EVENT_MESSAGE_GLOBAL_ENABLE_ASYNC_KEEP_ALIVE;
    uint8_t transportProtocolType = PLDM_TRANSPORT_PROTOCOL_TYPE_MCTP;
    /* default BMC EID is 8 */
    uint8_t eventReceiverAddressInfo = 0x08;
    uint16_t heartbeatTimer = 0x78;

    auto instanceId = requester.getInstanceId(eid);
    Request requestMsg(sizeof(pldm_msg_hdr) +
                       PLDM_SET_EVENT_RECEIVER_REQ_BYTES);

    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto rc = encode_set_event_receiver_req(
        instanceId, eventMessageGlobalEnable, transportProtocolType,
        eventReceiverAddressInfo, heartbeatTimer, request);
    if (rc != PLDM_SUCCESS)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "Failed to encode_set_event_receiver_req, rc = "
                  << unsigned(rc) << std::endl;
        co_return rc;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_PLATFORM)
                  << ", cmd= " << unsigned(PLDM_SET_EVENT_RECEIVER)
                  << ", rc=" << rc << std::endl;
        ;
        co_return rc;
    }

    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_PLATFORM)
                  << ", cmd= " << unsigned(PLDM_SET_EVENT_RECEIVER)
                  << ", rc=" << rc << std::endl;
        ;
        co_return rc;
    }

    rc = decode_set_event_receiver_resp(response, respMsgLen, &cc);

    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr << "Faile to decode_set_event_receiver_resp,"
                  << ", rc=" << unsigned(rc) << " cc=" << unsigned(cc)
                  << std::endl;
        co_return rc;
    }

    co_return cc;
}

void epochToBCDTime(const uint64_t& timeSec, uint8_t* seconds, uint8_t* minutes,
                    uint8_t* hours, uint8_t* day, uint8_t* month,
                    uint16_t* year)
{
    auto t = time_t(timeSec);
    auto time = localtime(&t);

    *seconds = (uint8_t)pldm::utils::decimalToBcd(time->tm_sec);
    *minutes = (uint8_t)pldm::utils::decimalToBcd(time->tm_min);
    *hours = (uint8_t)pldm::utils::decimalToBcd(time->tm_hour);
    *day = (uint8_t)pldm::utils::decimalToBcd(time->tm_mday);
    *month = (uint8_t)pldm::utils::decimalToBcd(
        time->tm_mon + 1); // The number of months in the range
                           // 0 to 11.PLDM expects range 1 to 12
    *year = (uint16_t)pldm::utils::decimalToBcd(
        time->tm_year + 1900); // The number of years since 1900
}

requester::Coroutine TerminusHandler::setDateTime()
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " update date time to terminus." << std::endl;
    uint8_t seconds = 0;
    uint8_t minutes = 0;
    uint8_t hours = 0;
    uint8_t day = 0;
    uint8_t month = 0;
    uint16_t year = 0;

    constexpr auto timeInterface = "xyz.openbmc_project.Time.EpochTime";
    constexpr auto bmcTimePath = "/xyz/openbmc_project/time/bmc";
    EpochTimeUS timeUsec;

    try
    {
        timeUsec = pldm::utils::DBusHandler().getDbusProperty<EpochTimeUS>(
            bmcTimePath, "Elapsed", timeInterface);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        std::cerr << "Error getting time, PATH=" << bmcTimePath
                  << " TIME INTERACE=" << timeInterface << std::endl;
        co_return PLDM_ERROR;
    }

    uint64_t timeSec = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::microseconds(timeUsec))
                           .count();

    epochToBCDTime(timeSec, &seconds, &minutes, &hours, &day, &month, &year);
    std::cerr << "SetDateTime timeUsec=" << timeUsec << " seconds="
              << unsigned(seconds) << " minutes=" << unsigned(minutes)
              << " hours=" << unsigned(hours) << " year=" << year << std::endl;

    std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) +
                                    sizeof(struct pldm_set_date_time_req));
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto instanceId = requester.getInstanceId(eid);

    auto rc = encode_set_date_time_req(instanceId, seconds, minutes, hours, day,
                                       month, year, request,
                                       sizeof(struct pldm_set_date_time_req));
    if (rc != PLDM_SUCCESS)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "Failed to encode_set_date_time_req, rc = " << unsigned(rc)
                  << std::endl;
        co_return PLDM_ERROR;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BIOS)
                  << ", cmd= " << unsigned(PLDM_SET_DATE_TIME)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_BIOS)
                  << ", cmd= " << unsigned(PLDM_SET_DATE_TIME)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    rc = decode_set_date_time_resp(response, respMsgLen, &cc);

    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr << "Response Message Error: "
                  << "rc=" << unsigned(rc) << ",cc=" << unsigned(cc)
                  << std::endl;
        co_return rc;
    }

    std::cerr << "Success SetDateTime to terminus " << devInfo.tid << std::endl;

    co_return cc;
}

std::string fruFieldValuestring(const uint8_t* value, const uint8_t& length)
{
    return std::string(reinterpret_cast<const char*>(value), length);
}

static uint32_t fruFieldParserU32(const uint8_t* value, const uint8_t& length)
{
    assert(length == 4);
    uint32_t v;
    std::memcpy(&v, value, length);
    return v;
}

static std::string fruFieldParserTimestamp(const uint8_t*, uint8_t)
{
    return std::string("TODO");
}

/** @brief Check if a pointer is go through end of table
 *  @param[in] table - pointer to FRU record table
 *  @param[in] p - pointer to each record of FRU record table
 *  @param[in] table_size - FRU table size
 */
bool isTableEnd(const uint8_t* table, const uint8_t* p, size_t& tableSize)
{
    auto offset = p - table;
    return (tableSize - offset) <= 7;
}

void TerminusHandler::parseFruRecordTable(const uint8_t* fruData,
                                          size_t& fruLen)
{
    std::string tidFRUObjPath;

    if (devInfo.tid == PLDM_TID_RESERVED)
    {
        std::cerr << "Invalid TID " << std::endl;
        return;
    }
    if (eidToName.second != "")
    {
        tidFRUObjPath = fruPath + "/" + eidToName.second;
    }
    else
    {
        tidFRUObjPath = fruPath + "/" + std::to_string(devInfo.tid);
    }

    auto fruPtr = std::make_shared<pldm::dbus_api::FruReq>(bus, tidFRUObjPath);
    frus.emplace(devInfo.tid, fruPtr);

    auto p = fruData;
    while (!isTableEnd(fruData, p, fruLen))
    {
        auto record = reinterpret_cast<const pldm_fru_record_data_format*>(p);

        p += sizeof(pldm_fru_record_data_format) - sizeof(pldm_fru_record_tlv);

        for (int i = 0; i < record->num_fru_fields; i++)
        {
            auto tlv = reinterpret_cast<const pldm_fru_record_tlv*>(p);
            if (record->record_type == PLDM_FRU_RECORD_TYPE_GENERAL)
            {
                switch (tlv->type)
                {
                    case PLDM_FRU_FIELD_TYPE_CHASSIS:
                        fruPtr->chassisType(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_MODEL:
                        fruPtr->model(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_PN:
                        fruPtr->pn(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_SN:
                        fruPtr->sn(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_MANUFAC:
                        fruPtr->manufacturer(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_MANUFAC_DATE:
                        fruPtr->manufacturerDate(
                            fruFieldParserTimestamp(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_VENDOR:
                        fruPtr->vendor(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_NAME:
                        fruPtr->name(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_SKU:
                        fruPtr->sku(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_VERSION:
                        fruPtr->version(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_ASSET_TAG:
                        fruPtr->assetTag(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_DESC:
                        fruPtr->description(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_EC_LVL:
                        fruPtr->ecLevel(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_OTHER:
                        fruPtr->other(
                            fruFieldValuestring(tlv->value, tlv->length));
                        break;
                    case PLDM_FRU_FIELD_TYPE_IANA:
                        fruPtr->iana(
                            fruFieldParserU32(tlv->value, tlv->length));
                        break;
                }
            }
            p += sizeof(pldm_fru_record_tlv) - 1 + tlv->length;
        }
    }
}

requester::Coroutine TerminusHandler::getFRURecordTableMetadata(uint16_t* total)
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " get FRU record Table Meta Data." << std::endl;
    auto instanceId = requester.getInstanceId(eid);
    Request requestMsg(sizeof(pldm_msg_hdr) +
                       PLDM_GET_FRU_RECORD_TABLE_METADATA_REQ_BYTES);
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto rc = encode_get_fru_record_table_metadata_req(
        instanceId, request, requestMsg.size() - sizeof(pldm_msg_hdr));
    if (rc != PLDM_SUCCESS)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "Failed to encode_get_fru_record_table_metadata_req, rc = "
                  << unsigned(rc) << std::endl;
        co_return rc;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_FRU)
                  << ", cmd= " << unsigned(PLDM_GET_FRU_RECORD_TABLE_METADATA)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_FRU)
                  << ", cmd= " << unsigned(PLDM_GET_FRU_RECORD_TABLE_METADATA)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t fru_data_major_version, fru_data_minor_version;
    uint32_t fru_table_maximum_size, fru_table_length;
    uint16_t total_record_set_identifiers;
    uint32_t checksum;
    rc = decode_get_fru_record_table_metadata_resp(
        response, respMsgLen, &cc, &fru_data_major_version,
        &fru_data_minor_version, &fru_table_maximum_size, &fru_table_length,
        &total_record_set_identifiers, total, &checksum);

    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr << "Faile to decode get fru record table metadata resp, "
                     "Message Error: "
                  << "rc=" << unsigned(rc) << ", cc=" << unsigned(cc)
                  << std::endl;
        co_return rc;
    }

    co_return rc;
}

requester::Coroutine
    TerminusHandler::getFRURecordTable(const uint16_t& totalTableRecords)
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " get FRU record Table." << std::endl;
    if (!totalTableRecords)
    {
        std::cerr << "Number of record table is not correct." << std::endl;
        co_return PLDM_ERROR;
    }

    auto instanceId = requester.getInstanceId(eid);
    Request requestMsg(sizeof(pldm_msg_hdr) +
                       PLDM_GET_FRU_RECORD_TABLE_REQ_BYTES);

    // send the getFruRecordTable command
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
    auto rc = encode_get_fru_record_table_req(
        instanceId, 0, PLDM_GET_FIRSTPART, request,
        requestMsg.size() - sizeof(pldm_msg_hdr));
    if (rc != PLDM_SUCCESS)
    {
        requester.markFree(eid, instanceId);
        std::cerr << "Failed to encode_get_fru_record_table_req, rc = "
                  << unsigned(rc) << std::endl;
        co_return rc;
    }

    Response responseMsg{};
    rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                             responseMsg);
    if (rc)
    {
        std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                  << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_FRU)
                  << ", cmd= " << unsigned(PLDM_GET_FRU_RECORD_TABLE)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint8_t cc = 0;
    auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
    auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "No response received for sendRecvPldmMsg, EID="
                  << unsigned(eid) << ", instanceId=" << unsigned(instanceId)
                  << ", type=" << unsigned(PLDM_FRU)
                  << ", cmd= " << unsigned(PLDM_GET_FRU_RECORD_TABLE)
                  << ", rc=" << unsigned(rc) << std::endl;
        ;
        co_return rc;
    }

    uint32_t nextDataTransferHandle = 0;
    uint8_t transferFlag = 0;
    size_t fruRecordTableLength = 0;
    std::vector<uint8_t> fruRecordTableData(respMsgLen - sizeof(pldm_msg_hdr));

    auto responsePtr = reinterpret_cast<const struct pldm_msg*>(response);
    rc = decode_get_fru_record_table_resp(
        responsePtr, respMsgLen - sizeof(pldm_msg_hdr), &cc,
        &nextDataTransferHandle, &transferFlag, fruRecordTableData.data(),
        &fruRecordTableLength);

    if (rc != PLDM_SUCCESS || cc != PLDM_SUCCESS)
    {
        std::cerr
            << "Failed to decode get fru record table resp, Message Error: "
            << "rc=" << unsigned(rc) << ", cc=" << unsigned(cc) << std::endl;
        co_return rc;
    }

    parseFruRecordTable(fruRecordTableData.data(), fruRecordTableLength);

    co_return cc;
}

requester::Coroutine TerminusHandler::getDevPDR(uint32_t nextRecordHandle)
{
    std::cerr << "Discovery Terminus: " << unsigned(eid)
              << " get terminus PDRs." << std::endl;
    do
    {
        std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) +
                                        PLDM_GET_PDR_REQ_BYTES);
        auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());
        uint32_t recordHandle{};
        if (nextRecordHandle)
        {
            recordHandle = nextRecordHandle;
        }
        auto instanceId = requester.getInstanceId(eid);

        auto rc =
            encode_get_pdr_req(instanceId, recordHandle, 0, PLDM_GET_FIRSTPART,
                               UINT16_MAX, 0, request, PLDM_GET_PDR_REQ_BYTES);
        if (rc != PLDM_SUCCESS)
        {
            requester.markFree(eid, instanceId);
            std::cerr << "Failed to encode_get_pdr_req, rc = " << unsigned(rc)
                      << std::endl;
            co_return rc;
        }

        Response responseMsg{};
        rc = co_await requester::sendRecvPldmMsg(*handler, eid, requestMsg,
                                                 responseMsg);
        if (rc)
        {
            std::cerr << "Failed to send sendRecvPldmMsg, EID=" << unsigned(eid)
                      << ", instanceId=" << unsigned(instanceId)
                      << ", type=" << unsigned(PLDM_PLATFORM)
                      << ", cmd= " << unsigned(PLDM_GET_PDR)
                      << ", rc=" << unsigned(rc) << std::endl;
            ;
            co_return rc;
        }

        auto respMsgLen = responseMsg.size() - sizeof(struct pldm_msg_hdr);
        auto response = reinterpret_cast<pldm_msg*>(responseMsg.data());
        if (response == nullptr || !respMsgLen)
        {
            std::cerr << "No response received for sendRecvPldmMsg, EID="
                      << unsigned(eid) << ", instanceId="
                      << unsigned(instanceId) << ", type="
                      << unsigned(PLDM_PLATFORM) << ", cmd= "
                      << unsigned(PLDM_GET_PDR) << ", rc="
                      << unsigned(rc) << std::endl;
            ;
            co_return rc;
        }
        rc = co_await processDevPDRs(eid, response, respMsgLen,
                                     &nextRecordHandle);
        if (rc)
        {
            std::cerr << "Failed to send processDevPDRs, EID=" << unsigned(eid)
                      << ", rc=" << unsigned(rc) << std::endl;
            ;
            co_return rc;
        }
    } while (nextRecordHandle != 0);

    if (!nextRecordHandle)
    {
        co_return PLDM_SUCCESS;
    }

    co_return PLDM_ERROR;
}

requester::Coroutine TerminusHandler::processDevPDRs(mctp_eid_t& /*eid*/,
                                                     const pldm_msg* response,
                                                     size_t& respMsgLen,
                                                     uint32_t* nextRecordHandle)
{
    uint8_t tlEid = 0;
    bool tlValid = true;
    uint32_t rh = 0;
    uint8_t tid = 0;

    uint8_t completionCode{};
    uint32_t nextDataTransferHandle{};
    uint8_t transferFlag{};
    uint16_t respCount{};
    uint8_t transferCRC{};
    if (response == nullptr || !respMsgLen)
    {
        std::cerr << "Failed to receive response for the GetPDR"
                     " command \n";
        co_return PLDM_ERROR;
    }

    auto rc = decode_get_pdr_resp(
        response, respMsgLen /*- sizeof(pldm_msg_hdr)*/, &completionCode,
        nextRecordHandle, &nextDataTransferHandle, &transferFlag, &respCount,
        nullptr, 0, &transferCRC);
    std::vector<uint8_t> responsePDRMsg;
    responsePDRMsg.resize(respMsgLen + sizeof(pldm_msg_hdr));
    memcpy(responsePDRMsg.data(), response, respMsgLen + sizeof(pldm_msg_hdr));
    if (rc != PLDM_SUCCESS)
    {
        std::cerr << "Failed to decode_get_pdr_resp, rc = " << unsigned(rc)
                  << std::endl;
        co_return rc;
    }

    std::vector<uint8_t> pdr(respCount, 0);
    rc = decode_get_pdr_resp(response, respMsgLen, &completionCode,
                             nextRecordHandle, &nextDataTransferHandle,
                             &transferFlag, &respCount, pdr.data(), respCount,
                             &transferCRC);

    if (rc != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
    {
        std::cerr << "Failed to decode_get_pdr_resp: "
                  << "rc=" << unsigned(rc)
                  << ", cc=" << unsigned(completionCode) << std::endl;
        co_return rc;
    }
    // when nextRecordHandle is 0, we need the recordHandle of the last
    // PDR and not 0-1.
    if (!(*nextRecordHandle))
    {
        rh = *nextRecordHandle;
    }
    else
    {
        rh = *nextRecordHandle - 1;
    }

    auto pdrHdr = reinterpret_cast<pldm_pdr_hdr*>(pdr.data());
    if (!rh)
    {
        rh = pdrHdr->record_handle;
    }

    if (pdrHdr->type == PLDM_PDR_ENTITY_ASSOCIATION)
    {
        /* Temporary remove merge Entity Association feature */
        this->mergeEntityAssociations(pdr);
        co_return PLDM_SUCCESS;
    }

    if (pdrHdr->type == PLDM_TERMINUS_LOCATOR_PDR)
    {
        auto tlpdr =
            reinterpret_cast<const pldm_terminus_locator_pdr*>(pdr.data());

        terminusHandle = tlpdr->terminus_handle;
        tid = tlpdr->tid;
        auto terminus_locator_type = tlpdr->terminus_locator_type;
        if (terminus_locator_type == PLDM_TERMINUS_LOCATOR_TYPE_MCTP_EID)
        {
            auto locatorValue =
                reinterpret_cast<const pldm_terminus_locator_type_mctp_eid*>(
                    tlpdr->terminus_locator_value);
            tlEid = static_cast<uint8_t>(locatorValue->eid);
        }
        if (tlpdr->validity == 0)
        {
            tlValid = false;
        }
        tlPDRInfo.insert_or_assign(
            tlpdr->terminus_handle,
            std::make_tuple(tlpdr->tid, tlEid, tlpdr->validity));
    }
    else if (pdrHdr->type == PLDM_COMPACT_NUMERIC_SENSOR_PDR)
    {
        this->compNumSensorPDRs.emplace_back(pdr);
    }
    else if (pdrHdr->type == PLDM_NUMERIC_EFFECTER_PDR)
    {
        this->effecterPDRs.emplace_back(pdr);
    }
    else if (pdrHdr->type == PLDM_EFFECTER_AUXILIARY_NAMES_PDR)
    {
        this->effecterAuxNamePDRs.emplace_back(pdr);
    }

    // if the TLPDR is invalid update the repo accordingly
    if (!tlValid)
    {
        pldm_pdr_update_TL_pdr(repo, terminusHandle, tid, tlEid, tlValid);
    }
    else
    {
        pldm_pdr_add_check(repo, pdr.data(), respCount, true,
                           terminusHandle, &rh);
    }

    co_return PLDM_SUCCESS;
}

void TerminusHandler::mergeEntityAssociations(const std::vector<uint8_t>& pdr)
{
    size_t numEntities{};
    pldm_entity* entities = nullptr;
    bool merged = false;
    auto entityPdr = reinterpret_cast<pldm_pdr_entity_association*>(
        const_cast<uint8_t*>(pdr.data()) + sizeof(pldm_pdr_hdr));

    pldm_entity_association_pdr_extract(pdr.data(), pdr.size(), &numEntities,
                                        &entities);
    for (size_t i = 0; i < numEntities; ++i)
    {
        pldm_entity parent{};
        if (getParent(entities[i].entity_type, &parent))
        {
            auto node = pldm_entity_association_tree_find(entityTree, &parent);
            if (node)
            {
                pldm_entity_association_tree_add(entityTree, &entities[i],
                                                 0xFFFF, node,
                                                 entityPdr->association_type);
                merged = true;
            }
        }
    }

    if (merged)
    {
        // Update our PDR repo with the merged entity association PDRs
        pldm_entity_node* node = nullptr;
        pldm_find_entity_ref_in_tree(entityTree, entities[0], &node);
        if (node == nullptr)
        {
            std::cerr
                << "\ncould not find referrence of the entity in the tree \n";
        }
        else
        {
            pldm_entity_association_pdr_add_from_node_check(node, repo,
                                                            &entities,
                                                            numEntities, true,
                                                            terminusHandle);
        }
    }
    free(entities);
}

bool TerminusHandler::getParent(const EntityType& type, pldm_entity* parent)
{
    auto found = parents.find(type);
    if (found != parents.end())
    {
        parent->entity_type = found->second.entity_type;
        parent->entity_instance_num = found->second.entity_instance_num;
        return true;
    }

    return false;
}

void TerminusHandler::createCompactNummericSensorIntf(const PDRList& sensorPDRs)
{
    /** @brief Store the added sensor D-Bus object path */
    std::vector<uint16_t> _addedSensorId;
    for (const auto& sensorPDR : sensorPDRs)
    {
        auto pdr = reinterpret_cast<const pldm_compact_numeric_sensor_pdr*>(
            sensorPDR.data());

        auto it = std::find(_addedSensorId.begin(), _addedSensorId.end(),
                            pdr->sensor_id);
        if (it != _addedSensorId.end())
        {
            std::cerr << "Sensor " << pdr->sensor_id << " added." << std::endl;
            continue;
        }
        _addedSensorId.emplace_back(pdr->sensor_id);

        PldmSensorInfo sensorInfo{};
        auto terminusHandle = pdr->terminus_handle;
        sensorInfo.entityType = pdr->entity_type;
        sensorInfo.entityInstance = pdr->entity_instance;
        sensorInfo.containerId = pdr->container_id;
        sensorInfo.sensorNameLength = pdr->sensor_name_length;
        if (sensorInfo.sensorNameLength == 0)
        {
            sensorInfo.sensorName =
                "SensorId" + std::to_string(unsigned(pdr->sensor_id));
        }
        else
        {
            std::string sTemp(reinterpret_cast<char const*>(pdr->sensor_name),
                              sensorInfo.sensorNameLength);
            size_t pos = 0;
            while ((pos = sTemp.find(" ")) != std::string::npos)
            {
                sTemp.replace(pos, 1, "_");
            }
            sensorInfo.sensorName = sTemp;
        }

        sensorInfo.baseUnit = pdr->base_unit;
        sensorInfo.unitModifier = pdr->unit_modifier;
        sensorInfo.offset = 0;
        sensorInfo.resolution = 1;
        sensorInfo.occurrenceRate = pdr->occurrence_rate;
        sensorInfo.rangeFieldSupport = pdr->range_field_support;
        sensorInfo.warningHigh = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.warningLow = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.criticalHigh = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.criticalLow = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.fatalHigh = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.fatalLow = std::numeric_limits<double>::quiet_NaN();
        if (pdr->range_field_support.bits.bit0)
        {
            sensorInfo.warningHigh = double(pdr->warning_high);
        }
        if (pdr->range_field_support.bits.bit1)
        {
            sensorInfo.warningLow = double(pdr->warning_low);
        }
        if (pdr->range_field_support.bits.bit2)
        {
            sensorInfo.criticalHigh = double(pdr->critical_high);
        }
        if (pdr->range_field_support.bits.bit3)
        {
            sensorInfo.criticalLow = double(pdr->critical_low);
        }
        if (pdr->range_field_support.bits.bit4)
        {
            sensorInfo.fatalHigh = double(pdr->fatal_high);
        }
        if (pdr->range_field_support.bits.bit5)
        {
            sensorInfo.fatalLow = double(pdr->fatal_low);
        }

        auto terminusId = PLDM_TID_RESERVED;
        try
        {
            terminusId = std::get<0>(tlPDRInfo.at(terminusHandle));
        }
        catch (const std::out_of_range& e)
        {
            // Do nothing
        }

        /* There is TID mapping */
        if (eidToName.second != "")
        {
            /* PREFIX */
            if (eidToName.first == true)
            {
                sensorInfo.sensorName =
                    eidToName.second + sensorInfo.sensorName;
            }
            else
            {
                sensorInfo.sensorName =
                    sensorInfo.sensorName + eidToName.second;
            }
        }
        else
        {
            sensorInfo.sensorName = sensorInfo.sensorName + "_TID" +
                                    std::to_string(unsigned(terminusId));
        }
        std::cerr << "Adding sensor name: " << sensorInfo.sensorName
                  << std::endl;

        auto sensorObject = std::make_unique<PldmSensor>(
            bus, sensorInfo.sensorName, sensorInfo.baseUnit,
            sensorInfo.unitModifier, sensorInfo.offset, sensorInfo.resolution,
            sensorInfo.warningHigh, sensorInfo.warningLow,
            sensorInfo.criticalHigh, sensorInfo.criticalLow);

        auto object = sensorObject->createSensor();
        if (object)
        {
            auto value =
                std::make_tuple(pdr->sensor_id, std::move((*object).second));
            auto key = std::make_tuple(eid, pdr->sensor_id, pdr->hdr.type);

            _sensorObjects[key] = std::move(sensorObject);
            _state[std::move(key)] = std::move(value);
        }
    }

    return;
}

void TerminusHandler::createNummericEffecterDBusIntf(const PDRList& sensorPDRs)
{
    std::vector<auxNameKey> _addedEffecter;
    for (const auto& sensorPDR : sensorPDRs)
    {
        auto pdr = reinterpret_cast<const pldm_numeric_effecter_value_pdr*>(
            sensorPDR.data());
        auxNameKey namekey =
            std::make_tuple(pdr->terminus_handle, pdr->effecter_id);

        auto it =
            std::find(_addedEffecter.begin(), _addedEffecter.end(), namekey);
        if (it != _addedEffecter.end())
        {
            std::cerr << "Effecter " << pdr->effecter_id << " existed."
                      << std::endl;
            continue;
        }
        _addedEffecter.emplace_back(namekey);

        PldmSensorInfo sensorInfo{};
        auto terminusHandle = pdr->terminus_handle;
        sensorInfo.entityType = pdr->entity_type;
        sensorInfo.entityInstance = pdr->entity_instance;
        sensorInfo.containerId = pdr->container_id;

        std::string sTemp = "";
        if (_auxNameMaps.find(namekey) != _auxNameMaps.end())
        {
            try
            {
                /* Use first name of first sensor idx for effecter name */
                sTemp = get<1>(_auxNameMaps[namekey][0][0]);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Failed to get name of Aux Name Key : "
                          << get<0>(namekey) << ":" << get<1>(namekey) << '\n';
                sTemp =
                    "Effecter_" + std::to_string(unsigned(pdr->effecter_id));
            }
        }
        else
        {
            std::cerr << "No Aux Name of effecter : " << get<0>(namekey) << ":"
                      << get<1>(namekey) << '\n';
            sTemp = "Effecter_" + std::to_string(unsigned(pdr->effecter_id));
        }

        size_t pos = 0;
        while ((pos = sTemp.find(" ")) != std::string::npos)
        {
            sTemp.replace(pos, 1, "_");
        }
        sensorInfo.sensorName = sTemp;
        sensorInfo.sensorNameLength = sTemp.length();

        sensorInfo.baseUnit = pdr->base_unit;
        sensorInfo.unitModifier = pdr->unit_modifier;
        sensorInfo.offset = pdr->offset;
        sensorInfo.resolution = pdr->resolution;
        sensorInfo.occurrenceRate = pdr->rate_unit;
        sensorInfo.rangeFieldSupport = pdr->range_field_support;
        sensorInfo.warningHigh = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.warningLow = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.criticalHigh = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.criticalLow = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.fatalHigh = std::numeric_limits<double>::quiet_NaN();
        sensorInfo.fatalLow = std::numeric_limits<double>::quiet_NaN();
        auto terminusId = PLDM_TID_RESERVED;
        try
        {
            terminusId = std::get<0>(tlPDRInfo.at(terminusHandle));
        }
        catch (const std::out_of_range& e)
        {
            // Do nothing
        }

        /* There is TID mapping */
        if (eidToName.second != "")
        {
            /* PREFIX */
            if (eidToName.first == true)
            {
                sensorInfo.sensorName =
                    eidToName.second + sensorInfo.sensorName;
            }
            else
            {
                sensorInfo.sensorName =
                    sensorInfo.sensorName + eidToName.second;
            }
        }
        else
        {
            sensorInfo.sensorName = sensorInfo.sensorName + "_TID" +
                                    std::to_string(unsigned(terminusId));
        }
        std::cerr << "Adding effecter name: " << sensorInfo.sensorName
                  << std::endl;

        auto sensorObj = std::make_unique<PldmSensor>(
            bus, sensorInfo.sensorName, sensorInfo.baseUnit,
            sensorInfo.unitModifier, sensorInfo.offset, sensorInfo.resolution,
            sensorInfo.warningHigh, sensorInfo.warningLow,
            sensorInfo.criticalHigh, sensorInfo.criticalLow);

        auto object = sensorObj->createSensor();
        if (object)
        {
            auto value =
                std::make_tuple(pdr->effecter_id, std::move((*object).second));
            auto key = std::make_tuple(eid, pdr->effecter_id, pdr->hdr.type);

            _sensorObjects[key] = std::move(sensorObj);
            _effecterLists.emplace_back(key);
            _state[std::move(key)] = std::move(value);
        }
    }

    return;
}

void TerminusHandler::parseAuxNamePDRs(const PDRList& sensorPDRs)
{
    constexpr uint8_t nullTerminator = 0;
    for (const auto& sensorPDR : sensorPDRs)
    {
        auto pdr =
            (struct pldm_sensor_auxiliary_names_pdr*)sensorPDR.data();

        if (!pdr)
        {
            std::cerr << "Failed to get Aux Name PDR" << std::endl;
            return;
        }
        auxNameKey key((uint16_t)pdr->terminus_handle,
                       (uint16_t)pdr->sensor_id);
        auxNameSensorMapping sensorNameMapping;

        const uint8_t* ptr = pdr->names;
        for ([[maybe_unused]] auto i :
             std::views::iota(0, (int)pdr->sensor_count))
        {
            const uint8_t nameStringCount = static_cast<uint8_t>(*ptr);
            auxNameList nameLists{};
            ptr += sizeof(uint8_t);
            for ([[maybe_unused]] auto j :
                 std::views::iota(0, (int)nameStringCount))
            {
                std::string nameLanguageTag(reinterpret_cast<const char*>(ptr),
                                            0, PLDM_STR_UTF_8_MAX_LEN);
                ptr += nameLanguageTag.size() + sizeof(nullTerminator);
                std::u16string u16NameString(
                    reinterpret_cast<const char16_t*>(ptr), 0,
                    PLDM_STR_UTF_16_MAX_LEN);
                ptr += (u16NameString.size() + sizeof(nullTerminator)) *
                       sizeof(uint16_t);
                std::transform(u16NameString.cbegin(), u16NameString.cend(),
                               u16NameString.begin(),
                               [](uint16_t utf16) { return be16toh(utf16); });
                std::string nameString =
                    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>,
                                         char16_t>{}
                        .to_bytes(u16NameString);
                nameLists.emplace_back(
                    std::make_tuple(nameLanguageTag, nameString));
            }
            if (!nameLists.size())
            {
                continue;
            }
            sensorNameMapping.emplace_back(nameLists);
        }

        if (!sensorNameMapping.size())
        {
            std::cerr << "Failed to find Aux Name of sensor Key " << get<0>(key)
                      << ":" << get<1>(key) << "in mapping table." << std::endl;
            continue;
        }
        if (_auxNameMaps.find(key) != _auxNameMaps.end())
        {
            std::cerr << "Aux Name Key : " << get<0>(key) << ":" << get<1>(key)
                      << " existed in mapping table." << std::endl;
            continue;
        }
        _auxNameMaps[key] = sensorNameMapping;
    }
    return;
}

} // namespace terminus

} // namespace pldm
