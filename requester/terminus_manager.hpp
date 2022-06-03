#pragma once

#include "pldmd/dbus_impl_requester.hpp"
#include "pldmd/instance_id.hpp"
#include "requester/handler.hpp"
#include "requester/request.hpp"
#include "requester/terminus_handler.hpp"

#include <nlohmann/json.hpp>
#include <sdeventplus/event.hpp>

#include <filesystem>
#include <iostream>
#include <map>
#include <vector>

namespace pldm
{

namespace terminus
{

using namespace pldm::dbus_api;
using namespace pldm::terminus;
namespace fs = std::filesystem;
using Json = nlohmann::json;

/** @class Manager
 *
 * This class manages the PLDM terminus
 */
class Manager
{

  public:
    Manager() = delete;
    Manager(const Manager&) = delete;
    Manager(Manager&&) = delete;
    Manager& operator=(const Manager&) = delete;
    Manager& operator=(Manager&&) = delete;
    ~Manager() = default;

    /** @brief Constructor
     *
     *  @param[in] handler - PLDM request handler
     */
    explicit Manager(
        sdbusplus::bus::bus& bus, sdeventplus::Event& event, pldm_pdr* repo,
        pldm_entity_association_tree* entityTree,
        pldm_entity_association_tree* bmcEntityTree,
        pldm::requester::Handler<pldm::requester::Request>* handler,
        pldm::InstanceIdDb& instanceIdDb) :
        bus(bus),
        event(event), repo(repo), entityTree(entityTree),
        bmcEntityTree(bmcEntityTree), handler(handler),
        instanceIdDb(instanceIdDb)
    {
        if (!setupEIDtoTeminusName(EID_TO_NAME_JSON))
        {
            std::cerr << "Failed to set up EID To TerminusName." << std::endl;
        }
    }

    /** @brief Add the discovered MCTP endpoints to the managed devices list
     *
     *  @param[in] eids - Array of MCTP endpoints
     *
     *  @return None
     */
    void addDevices(const std::vector<mctp_eid_t>& eids)
    {
        for (const auto& it : eids)
        {
            std::cerr << "Adding terminus EID : " << unsigned(it) << std::endl;

            auto dev = std::make_unique<TerminusHandler>(
                it, event, bus, repo, entityTree, bmcEntityTree,
                handler, instanceIdDb);

            std::pair<bool, std::string> eidMap = std::make_pair(true, "");
            if (eidToNameMaps.count(it))
            {
                eidMap = eidToNameMaps[it];
            }
            dev->udpateEidMapping(eidMap);
            dev->discoveryTerminus();
            dev->updateSensor();
            mDevices[it] = std::move(dev);
        }
        return;
    }

    /** @brief Remove the MCTP devices from the managed devices list
     *
     *  @param[in] eids - Array of MCTP endpoints
     *
     *  @return None
     */
    void removeDevices(const std::vector<mctp_eid_t>& eids)
    {
        for (const auto& it : eids)
        {
            std::cerr << "Removing Device EID : " << unsigned(it) << std::endl;
            if (!mDevices.count(it))
            {
                continue;
            }
            mDevices.erase(it);
        }
        return;
    }

  private:
    /** @brief reference of main D-bus interface of pldmd devices */
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

    std::map<mctp_eid_t, std::unique_ptr<TerminusHandler>> mDevices;

    /*
     * Mapping from "TIDx" in sensor name to prefix/subfix "ABCD"
     */
    std::map<uint8_t, std::pair<bool, std::string>> eidToNameMaps;

    bool setupEIDtoTeminusName(const fs::path& path)
    {
        const Json emptyJson{};
        if (!fs::exists(path))
        {
            std::cerr << path << "is not existing." << std::endl;
        }
        std::ifstream jsonFile(path);
        auto datas = Json::parse(jsonFile, nullptr, false);
        if (datas.is_discarded())
        {
            std::cerr << "Parsing eid to String Map config file failed, FILE="
                      << path;
            std::abort();
        }

        auto entries = datas.value("eids", emptyJson);
        for (const auto& entry : entries)
        {
            try
            {
                auto eid = entry.value("eid", -1);
                if (eid == -1)
                {
                    std::cerr << "Invalid \"eid\" configuration" << std::endl;
                    continue;
                }
                auto mapString = entry.value("string", "");
                if (mapString == "")
                {
                    std::cerr << "Invalid configuration of \"string\" of eid "
                              << unsigned(eid) << std::endl;
                    continue;
                }
                auto isPrefix = entry.value("prefix", true);
                eidToNameMaps[eid] = std::make_pair(isPrefix, mapString);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Terminus eid to String lookup map format error\n";
                continue;
            }
        }

        return true;
    }
};

}; // namespace terminus

} // namespace pldm
