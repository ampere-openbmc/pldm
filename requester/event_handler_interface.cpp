#include "config.h"
#include "libpldm/requester/pldm.h"
#include "event_hander_interface.hpp"

#include <assert.h>
#include <systemd/sd-journal.h>

#include <nlohmann/json.hpp>
#include <sdbusplus/timer.hpp>
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

using namespace pldm::utils;

#undef DEBUG
#define MAX_ATTEMPT 3

namespace pldm
{
EventHandlerInterface::EventHandlerInterface(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm::dbus_api::Requester& requester,
    pldm::requester::Handler<pldm::requester::Request>* handler) :
    eid(eid), bus(bus), event(event), requester(requester),
    handler(handler),
    normEventTimer(event, std::bind(&EventHandlerInterface::normalEventCb, this)),
    critEventTimer(event, std::bind(&EventHandlerInterface::criticalEventCb, this)),
    pollEventReqTimer(event, std::bind(&EventHandlerInterface::pollEventReqCb, this))
{
    pollReqTimeoutTimer = std::make_unique<phosphor::Timer>(
                                 [&](void) { pollReqTimeoutHdl(); });
    startCallback();
}

void EventHandlerInterface::normalEventCb()
{
    if (isProcessPolling || isCritical)
        return;

    /* Periodically poll for dummy RAS event data */
    uint16_t eventId = 0x0;
    /* prepare request data */
    reqData.operationFlag = PLDM_GET_FIRSTPART;
    reqData.dataTransferHandle = eventId;
    reqData.eventIdToAck = eventId;
#ifdef DEBUG
            std::cout << "\nHandle Normal EVENT_ID " << std::hex << eventId << "\n";
#endif
      pollEventReqTimer.restart(std::chrono::milliseconds(POLL_REQ_EVENT_TIMER));
}

void EventHandlerInterface::criticalEventCb()
{
    if (isProcessPolling)
        return;
    if (critEventQueue.empty())
    {
        isCritical = false;
        return;
    }
    /* Has Critical Event */
    isCritical = true;
    uint16_t eventId = critEventQueue.front();
    critEventQueue.pop_front();
    /* prepare request data */
    reqData.operationFlag = PLDM_GET_FIRSTPART;
    reqData.dataTransferHandle = eventId;
    reqData.eventIdToAck = eventId;
#ifdef DEBUG
            std::cout << "\nHandle Critical EVENT_ID " << std::hex << eventId << "\n";
#endif
      pollEventReqTimer.restart(std::chrono::milliseconds(POLL_REQ_EVENT_TIMER));
}

int EventHandlerInterface::enqueueCriticalEvent(uint16_t item)
{
    if (critEventQueue.size() > MAX_QUEUE_SIZE)
        return -1;

    for (auto& i : critEventQueue)
    {
        if (i == item)
        {
            return -2;
        }
    }

#ifdef DEBUG
    std::cout << "\nQUEUING CRIT EVENT_ID " << std::hex << item << "\n";
#endif
    // insert to event queue
    critEventQueue.push_back(item);

    return 0;
}

void EventHandlerInterface::pollReqTimeoutHdl()
{
    if (!responseReceived)
    {
#ifdef DEBUG
        std::cout << "POLL REQ TIMEOUT DROP EVENT_ID \n" << std::hex
                  << reqData.eventIdToAck << "\n";
#endif
        // clear cached data
        reset();
    }
}

void EventHandlerInterface::registerEventHandler(uint8_t eventClass,
                                                 HandlerFunc function)
{
    eventHndls.emplace(eventClass, function);
}

void EventHandlerInterface::reset()
{
    isProcessPolling = false;
    isPolling = false;
    responseReceived = false;
    memset(&reqData, 0, sizeof(struct ReqPollInfo));
    recvData.eventClass = 0;
    recvData.totalSize = 0;
    recvData.data.clear();
    requester.markFree(eid, instanceId);
    pollEventReqTimer.setEnabled(false);
}

void EventHandlerInterface::processResponseMsg(mctp_eid_t /*eid*/,
                                               const pldm_msg* response,
                                               size_t respMsgLen)
{
    uint8_t retCompletionCode;
    uint8_t retTid{};
    uint16_t retEventId;
    uint32_t retNextDataTransferHandle{};
    uint8_t retTransferFlag{};
    uint8_t retEventClass{};
    uint32_t retEventDataSize{};
    uint32_t retEventDataIntegrityChecksum{};

    // announce that data is received
    responseReceived = true;
    isPolling = false;
    pollReqTimeoutTimer->stop();

    std::vector<uint8_t> tmp(respMsgLen, 0);

    auto rc = decode_poll_for_platform_event_message_resp(
        response, respMsgLen, &retCompletionCode, &retTid, &retEventId,
        &retNextDataTransferHandle, &retTransferFlag, &retEventClass,
        &retEventDataSize, tmp.data(), &retEventDataIntegrityChecksum);
    if (rc != PLDM_SUCCESS)
    {
#ifdef DEBUG
        std::cerr
            << "ERROR: Failed to decode_poll_for_platform_event_message_resp, rc = "
            << rc << std::endl;
#endif
        reset();
        return;
    }

    retEventId &= 0xffff;
    retNextDataTransferHandle &= 0xffffffff;

#ifdef DEBUG
    std::cout << "\nRESPONSE: \n"
              << "retTid: " << std::hex << (unsigned)retTid << "\n"
              << "retEventId: " << std::hex << retEventId << "\n"
              << "retNextDataTransferHandle: " << std::hex
              << retNextDataTransferHandle << "\n"
              << "retTransferFlag: " << std::hex << (unsigned)retTransferFlag
              << "\n"
              << "retEventClass: " << std::hex << (unsigned)retEventClass
              << "\n"
              << "retEventDataSize: " << retEventDataSize << "\n"
              << "retEventDataIntegrityChecksum: " << std::hex
              << retEventDataIntegrityChecksum << "\n";
#endif

    if (retEventId == 0x0 || retEventId == 0xffff)
    {
        reset();
        return;
    }

    // drop if response eventId doesn't match with request eventId
    if ((reqData.eventIdToAck != 0x0) && (retEventId != reqData.eventIdToAck))
    {
#ifdef DEBUG
        std::cerr << "WARNING: RESPONSED EVENT_ID DOESN'T MATCH WITH QUEUING\n"
                  << "Recv EvenID=" << std::hex << retEventId
                  << "Req EvenID=" << std::hex << reqData.eventIdToAck
                  << "\n";
#endif
        reset();
        return;
    }


    // found
    int flag = static_cast<int>(retTransferFlag);

    if (flag == PLDM_START)            /* Start part */
    {
        recvData.data.insert(recvData.data.begin(),
                             tmp.begin(), tmp.begin() + retEventDataSize);
        recvData.totalSize += retEventDataSize;
        reqData.operationFlag = PLDM_GET_NEXTPART;
        reqData.dataTransferHandle = retNextDataTransferHandle;
        reqData.eventIdToAck = retEventId;
    }
    else if (flag == PLDM_MIDDLE)       /* Middle part */
    {
        recvData.data.insert(recvData.data.begin() + reqData.dataTransferHandle,
                             tmp.begin(), tmp.begin() + retEventDataSize);
        recvData.totalSize += retEventDataSize;
        reqData.operationFlag = PLDM_GET_NEXTPART;
        reqData.dataTransferHandle = retNextDataTransferHandle;
        reqData.eventIdToAck = retEventId;
    }
    else if ((flag == PLDM_END) || (flag == PLDM_START_AND_END))  /* End part */
    {
        recvData.data.insert(recvData.data.begin() + reqData.dataTransferHandle,
                             tmp.begin(), tmp.begin() + retEventDataSize);
        recvData.totalSize += retEventDataSize;

        /* eventDataIntegrityChecksum field is only used for multi-part transfer.
         * If single-part, ignore checksum.
         */
        uint32_t checksum = crc32(recvData.data.data(), recvData.data.size());
        if ((flag == PLDM_END) && (checksum != retEventDataIntegrityChecksum))
        {
            std::cerr << "\nchecksum isn't correct chks=" << std::hex << checksum
                      << " eventDataCRC=" << std::hex
                      << retEventDataIntegrityChecksum << "\n ";
        }
        else
        {
            // invoke class handler
            auto it = eventHndls.find(retEventClass);
            if (it != eventHndls.end())
            {
                eventHndls.at(retEventClass)(retTid, retEventClass,
                            retEventId, recvData.data);
            }
        }

        reqData.operationFlag = PLDM_ACKNOWLEDGEMENT_ONLY;
        reqData.dataTransferHandle = 0;
        reqData.eventIdToAck = retEventId;
    }
#ifdef DEBUG
        std::cout << "\nEVENT_ID:" << retEventId
                  << " DATA LENGTH:" << recvData.totalSize << "\n ";
        for (auto it = recvData.data.begin(); it != recvData.data.end(); it++)
        {
            std::cout << std::setfill('0') << std::setw(2) << std::hex
                      << (unsigned)*it << " ";
        }
        std::cout << "\n";
#endif

}

void EventHandlerInterface::pollEventReqCb()
{
    std::vector<uint8_t> requestMsg(sizeof(pldm_msg_hdr) +
                         PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES);
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());

    if (isPolling)
        return;

    if (reqData.eventIdToAck == 0xffff)
        return;

#ifdef DEBUG
    std::cout << "\nREQUEST \n"
              << "TransferoperationFlag: " << std::hex << (unsigned)reqData.operationFlag << "\n"
              << "eventIdToAck: " << std::hex << reqData.eventIdToAck << "\n"
              << "dataTransferHandle: " << std::hex << reqData.dataTransferHandle << "\n";
#endif

    instanceId = requester.getInstanceId(eid);
    auto rc = encode_poll_for_platform_event_message_req(
                    instanceId, 1, reqData.operationFlag,
                    reqData.dataTransferHandle, reqData.eventIdToAck,
                    request, PLDM_POLL_FOR_PLATFORM_EVENT_MESSAGE_REQ_BYTES);

    if (rc != PLDM_SUCCESS)
    {
        std::cerr
            << "ERROR: Failed to encode_poll_for_platform_event_message_req(1), rc = "
            << rc << std::endl;
        return;
    }

    rc = handler->registerRequest(
        eid, instanceId, PLDM_PLATFORM, PLDM_POLL_FOR_EVENT_MESSAGE,
        std::move(requestMsg),
        std::move(
            std::bind_front(&EventHandlerInterface::processResponseMsg, this)));
    if (rc)
    {
        std::cerr << "ERROR: failed to send the poll request\n";
        requester.markFree(eid, instanceId);
        return;
    }

    // flags settings
    isProcessPolling = true;
    isPolling = true;
    responseReceived = false;
    pollReqTimeoutTimer->start(std::chrono::milliseconds(
                         (NUMBER_OF_REQUEST_RETRIES + 1) * RESPONSE_TIME_OUT));

}

void EventHandlerInterface::startCallback()
{
    std::function<void()> normalCb(
        std::bind(&EventHandlerInterface::normalEventCb, this));
    std::function<void()> criticalCb(
            std::bind(&EventHandlerInterface::criticalEventCb, this));
    try
    {
        normEventTimer.restart(std::chrono::milliseconds(NORMAL_RAS_EVENT_TIMER));
        critEventTimer.restart(std::chrono::milliseconds(CRITICAL_RAS_EVENT_TIMER));
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: cannot start callback for normal"
                     " and critical event" << std::endl;
        throw;
    }
}

void EventHandlerInterface::stopCallback()
{
    try
    {
        normEventTimer.setEnabled(false);
        critEventTimer.setEnabled(false);
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: cannot stop callback" << std::endl;
        throw;
    }
}

} // namespace pldm
