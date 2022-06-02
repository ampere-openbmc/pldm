#include "config.h"

#include "terminus_handler.hpp"

namespace pldm
{

namespace terminus
{

TerminusHandler::TerminusHandler(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm_pdr* repo, pldm_entity_association_tree* entityTree,
    pldm_entity_association_tree* bmcEntityTree,
    pldm::requester::Handler<pldm::requester::Request>* handler,
    pldm::InstanceIdDb& instanceIdDb) :
    eid(eid),
    bus(bus), event(event), repo(repo), entityTree(entityTree),
    bmcEntityTree(bmcEntityTree), handler(handler),
    instanceIdDb(instanceIdDb)
{}

TerminusHandler::~TerminusHandler()
{}

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

    if (supportPLDMType(PLDM_PLATFORM))
    {
        rc = co_await setEventReceiver();
        if (rc)
        {
            std::cerr << "Failed to setEventReceiver, rc=" << unsigned(rc)
                      << std::endl;
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

} // namespace terminus

} // namespace pldm
