#pragma once

#include "common/types.hpp"
#include "pldmd/dbus_impl_requester.hpp"
#include "requester/handler.hpp"

#include <sdeventplus/event.hpp>
#include <sdeventplus/source/event.hpp>

#include <map>

namespace pldm
{

namespace terminus
{

using namespace pldm::dbus_api;

/** @class TerminusHandler
 *  @brief This class can fetch and process PDRs from host firmware
 *  @details Provides an API to fetch PDRs from the host firmware. Upon
 *  receiving the PDRs, they are stored into the BMC's primary PDR repo.
 *  Adjustments are made to entity association PDRs received from the host,
 *  because they need to be assimilated into the BMC's entity association
 *  tree. A PLDM event containing the record handles of the updated entity
 *  association PDRs is sent to the host.
 */
class TerminusHandler
{
  public:
    using TerminusInfo =
        std::tuple<pdr::TerminusID, pdr::EID, pdr::TerminusValidity>;
    using TLPDRMap = std::map<pdr::TerminusHandle, TerminusInfo>;

    /** @brief Constructor
     *  @param[in] eid - MCTP EID of host firmware
     *  @param[in] event - reference of main event loop of pldmd
     *  @param[in] repo - pointer to BMC's primary PDR repo
     *  @param[in] eventsJsonDir - directory path which has the config JSONs
     *  @param[in] entityTree - Pointer to BMC and Host entity association tree
     *  @param[in] bmcEntityTree - pointer to BMC's entity association tree
     *  @param[in] handler - PLDM request handler
     *  @param[in] instanceIdDb - PLDM instance ID data base
     */
    explicit TerminusHandler(
        uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
        pldm_pdr* repo, pldm_entity_association_tree* entityTree,
        pldm_entity_association_tree* bmcEntityTree,
        pldm::requester::Handler<pldm::requester::Request>* handler,
        pldm::InstanceIdDb& instanceIdDb);

    TerminusHandler(const TerminusHandler&) = delete;
    TerminusHandler& operator=(const TerminusHandler&) = delete;
    TerminusHandler(TerminusHandler&&) = delete;
    TerminusHandler& operator=(TerminusHandler&&) = delete;

    ~TerminusHandler();

    /** @brief check whether terminus is running when pldmd starts
     */
    bool isTerminusOn()
    {
        return responseReceived;
    }

    /** @brief Discovery new terminus
     *
     * @return - none
     */
    void discoveryTerminus();

  private:
    /** @brief map that captures various terminus information **/
    TLPDRMap tlPDRInfo;

    /** @brief MCTP EID of host firmware */
    uint8_t eid;
    /** @brief reference of main D-bus interface of pldmd terminus */
    sdbusplus::bus::bus& bus;

    /** @brief reference of main event loop of pldmd, primarily used to schedule
     *  work.
     */
    sdeventplus::Event& event;
    /** @brief pointer to BMC's primary PDR repo, host PDRs are added here */
    pldm_pdr* repo;

    /** @brief Pointer to BMC's and Host's entity association tree */
    pldm_entity_association_tree* entityTree;

    /** @brief Pointer to BMC's entity association tree */
    pldm_entity_association_tree* bmcEntityTree;

    /** @brief PLDM request handler */
    pldm::requester::Handler<pldm::requester::Request>* handler;

    /** @brief Instance ID database for managing instance ID*/
    InstanceIdDb& instanceIdDb;

    /** @brief whether response received from Host */
    bool responseReceived;
};

} // namespace terminus

} // namespace pldm
