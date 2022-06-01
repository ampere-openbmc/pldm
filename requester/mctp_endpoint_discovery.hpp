#pragma once

#include "fw-update/manager.hpp"
#include "requester/terminus_manager.hpp"

#include <sdbusplus/bus/match.hpp>

namespace pldm
{

constexpr auto MCTPService = "xyz.openbmc_project.MCTP";
constexpr auto MCTPPath = "/xyz/openbmc_project/mctp";

class MctpDiscovery
{
  public:
    MctpDiscovery() = delete;
    MctpDiscovery(const MctpDiscovery&) = delete;
    MctpDiscovery(MctpDiscovery&&) = delete;
    MctpDiscovery& operator=(const MctpDiscovery&) = delete;
    MctpDiscovery& operator=(MctpDiscovery&&) = delete;
    ~MctpDiscovery() = default;

    /** @brief Constructs the MCTP Discovery object to handle discovery of
     *         MCTP enabled devices
     *
     *  @param[in] bus - reference to systemd bus
     *  @param[in] fwManager - pointer to the firmware manager
     */
    explicit MctpDiscovery(sdbusplus::bus_t& bus,
                           fw_update::Manager* fwManager,
                           terminus::Manager* devManager);

  private:
    /** @brief reference to the systemd bus */
    sdbusplus::bus_t& bus;

    fw_update::Manager* fwManager;

    terminus::Manager* devManager;

    /** @brief Used to watch for new MCTP endpoints */
    sdbusplus::bus::match_t mctpEndpointAddedSignal;

    /** @brief Used to watch for the removed MCTP endpoints */
    sdbusplus::bus::match_t mctpEndpointRemovedSignal;

    void dicoverEndpoints(sdbusplus::message_t& msg);

    void removeEndpoints(sdbusplus::message_t& msg);

    static constexpr uint8_t mctpTypePLDM = 1;

    static constexpr std::string_view mctpEndpointIntfName{
        "xyz.openbmc_project.MCTP.Endpoint"};

    /* List MCTP endpoint in MCTP D-Bus interface or Static EID table */
    std::vector<mctp_eid_t> listEids;
};

} // namespace pldm
