#include "manager.hpp"

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace pldm
{
namespace platform_mc
{
exec::task<int> Manager::beforeDiscoverTerminus()
{
    // Add any setup or checks needed before discovering a terminus
    // If any setup/check fails, return the appropriate error code
    // For now, we assume everything is successful
    co_return PLDM_SUCCESS;
}

exec::task<int> Manager::afterDiscoverTerminus()
{
    auto rc = co_await platformManager.setTerminiNames(eidToMctpMediums);
    if (rc != PLDM_SUCCESS)
    {
        lg2::error("Failed to initialize termini names, error {RC}", "RC", rc);
    }

    rc = co_await platformManager.initTerminus();
    if (rc != PLDM_SUCCESS)
    {
        lg2::error("Failed to initialize platform manager, error {RC}", "RC",
                   rc);
    }
    else
    {
        lg2::info("Successfully initialized platform manager");
    }
    co_return rc;
}

exec::task<int> Manager::reconfigEventReceiver(pldm_tid_t tid)
{
    auto rc = co_await platformManager.configEventReceiver(tid);
    if (rc)
    {
        lg2::error(
            "Failed to reconfigure event receiver for terminus {TID}, error {RC}",
            "TID", tid, "RC", rc);
    }

    lg2::info("Successfully reconfigure event receiver for terminus {TID}.",
              "TID", tid);
    co_return PLDM_SUCCESS;
}

exec::task<int> Manager::pollForPlatformEvent(
    pldm_tid_t tid, uint16_t pollEventId, uint32_t pollDataTransferHandle)
{
    auto it = termini.find(tid);
    if (it != termini.end())
    {
        auto& terminus = it->second;
        co_await eventManager.pollForPlatformEventTask(tid, pollEventId,
                                                       pollDataTransferHandle);
        terminus->pollEvent = false;
    }
    co_return PLDM_SUCCESS;
}

exec::task<int> Manager::oemPollForPlatformEvent(pldm_tid_t tid)
{
    for (auto& handler : pollHandlers)
    {
        if (handler)
        {
            co_await handler(tid);
        }
    }
    co_return PLDM_SUCCESS;
}

void Manager::loadEidToMCTPMediumConfigs()
{
    if (!std::filesystem::exists(EID_TO_MCTP_MEDIUM))
    {
        lg2::error("File MCTP medium configuration {PATH} does not exist.",
                   "PATH", static_cast<std::string>(EID_TO_MCTP_MEDIUM));
        return;
    }

    static const std::vector<nlohmann::json> emptyJsonArray{};
    std::ifstream jsonFile(EID_TO_MCTP_MEDIUM);
    auto datas = nlohmann::json::parse(jsonFile, nullptr, false);
    if (datas.is_discarded())
    {
        lg2::error("Failed to parse MCTP medium configuration {PATH}.", "PATH",
                   static_cast<std::string>(EID_TO_MCTP_MEDIUM));
        return;
    }

    auto entries = datas.value("eids", emptyJsonArray);
    for (const auto& entry : entries)
    {
        auto eid = entry.value("eid", -1);
        if (eid == -1)
        {
            error("Invalid \"eid\" configuration");
            continue;
        }
        std::string mapString = entry.value("string", "");
        if (mapString == "")
        {
            lg2::error(
                "Invalid configuration of \"string\" of endpoint ID {EID} ",
                "EID", eid);
            continue;
        }

        if (mapString.substr(mapString.size() - 1, mapString.size()) == "_")
        {
            mapString = mapString.substr(0, mapString.size() - 1);
        }
        eidToMctpMediums[eid] = mapString;
    }

    return;
}

} // namespace platform_mc
} // namespace pldm
