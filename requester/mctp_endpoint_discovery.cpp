#include "config.h"

#include "mctp_endpoint_discovery.hpp"

#include "common/types.hpp"
#include "common/utils.hpp"

#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <string_view>
#include <vector>

using namespace sdbusplus::bus::match::rules;

PHOSPHOR_LOG2_USING;

namespace pldm
{
MctpDiscovery::MctpDiscovery(
    sdbusplus::bus_t& bus,
    std::initializer_list<MctpDiscoveryHandlerIntf*> list) :
    bus(bus),
    mctpEndpointPropChangedSignal(
        bus, propertiesChangedNamespace(MCTPPath, MCTPInterfaceCC),
        std::bind_front(&MctpDiscovery::propertiesChangedCb, this)),
    mctpEndpointAddedSignal(
        bus, interfacesAdded(MCTPPath),
        std::bind_front(&MctpDiscovery::discoverEndpoints, this)),
    mctpEndpointRemovedSignal(
        bus, interfacesRemoved(MCTPPath),
        std::bind_front(&MctpDiscovery::removeEndpoints, this)),
    handlers(list)
{
    std::map<MctpInfo, Availability> currentMctpInfoMap;
    getMctpInfos(currentMctpInfoMap);
    for (const auto& mapIt : currentMctpInfoMap)
    {
        if (mapIt.second)
        {
            // Only add the available endpoints to the terminus
            // Let the propertiesChanged signal tells us when it comes back
            // to Available again
            addToExistingMctpInfos(MctpInfos(1, mapIt.first));
        }
    }
    handleMctpEndpoints(existingMctpInfos);
}

void MctpDiscovery::getMctpInfos(std::map<MctpInfo, Availability>& mctpInfoMap)
{
    // Find all implementations of the MCTP Endpoint interface
    pldm::utils::GetSubTreeResponse mapperResponse;
    try
    {
        mapperResponse = pldm::utils::DBusHandler().getSubtree(
            MCTPPath, 0, std::vector<std::string>({MCTPInterface}));
    }
    catch (const sdbusplus::exception_t& e)
    {
        error(
            "Failed to getSubtree call at path '{PATH}' and interface '{INTERFACE}', error - {ERROR} ",
            "ERROR", e, "PATH", MCTPPath, "INTERFACE", MCTPInterface);
        return;
    }

    for (const auto& [path, services] : mapperResponse)
    {
        for (const auto& serviceIter : services)
        {
            const std::string& service = serviceIter.first;
            try
            {
                auto properties =
                    pldm::utils::DBusHandler().getDbusPropertiesVariant(
                        service.c_str(), path.c_str(), MCTPInterface);

                if (properties.contains("NetworkId") &&
                    properties.contains("EID") &&
                    properties.contains("SupportedMessageTypes"))
                {
                    pldm::utils::PropertyValue propertyValue =
                        pldm::utils::DBusHandler().getDbusPropertyVariant(
                            path.c_str(), MCTPConnectivityProp,
                            MCTPInterfaceCC);

                    auto availability = std::get<std::string>(propertyValue) ==
                                                "Available"
                                            ? true
                                            : false;
                    auto networkId =
                        std::get<NetworkId>(properties.at("NetworkId"));
                    auto eid = std::get<mctp_eid_t>(properties.at("EID"));
                    auto types = std::get<std::vector<uint8_t>>(
                        properties.at("SupportedMessageTypes"));
                    if (std::find(types.begin(), types.end(), mctpTypePLDM) !=
                        types.end())
                    {
                        lg2::info(
                            "Adding Endpoint networkId '{NETWORK}' and EID '{EID}'",
                            "NETWORK", networkId, "EID", eid);
                        mctpInfoMap[MctpInfo(eid, emptyUUID, "", networkId)] =
                            availability;
                    }
                }
            }
            catch (const sdbusplus::exception_t& e)
            {
                error(
                    "Error reading MCTP Endpoint property at path '{PATH}' and service '{SERVICE}', error - {ERROR}",
                    "ERROR", e, "SERVICE", service, "PATH", path);
                return;
            }
        }
    }
}

void MctpDiscovery::getAddedMctpInfos(sdbusplus::message_t& msg,
                                      MctpInfos& mctpInfos)
{
    using ObjectPath = sdbusplus::message::object_path;
    ObjectPath objPath;
    using Property = std::string;
    using PropertyMap = std::map<Property, dbus::Value>;
    std::map<std::string, PropertyMap> interfaces;

    try
    {
        msg.read(objPath, interfaces);
    }
    catch (const sdbusplus::exception_t& e)
    {
        error(
            "Error reading MCTP Endpoint added interface message, error - {ERROR}",
            "ERROR", e);
        return;
    }

    for (const auto& [intfName, properties] : interfaces)
    {
        if (intfName == MCTPInterface)
        {
            if (properties.contains("NetworkId") &&
                properties.contains("EID") &&
                properties.contains("SupportedMessageTypes"))
            {
                pldm::utils::PropertyValue propertyValue =
                    pldm::utils::DBusHandler().getDbusPropertyVariant(
                        objPath.str.c_str(), MCTPConnectivityProp,
                        MCTPInterfaceCC);

                auto availability =
                    std::get<std::string>(propertyValue) == "Available" ? true
                                                                        : false;
                auto networkId =
                    std::get<NetworkId>(properties.at("NetworkId"));
                auto eid = std::get<mctp_eid_t>(properties.at("EID"));
                auto types = std::get<std::vector<uint8_t>>(
                    properties.at("SupportedMessageTypes"));

                if (!availability)
                {
                    // Log an error message here, but still add it to the
                    // terminus
                    error(
                        "mctpd added a DEGRADED endpoint {EID} networkId {NET} to D-Bus",
                        "NET", networkId, "EID", static_cast<unsigned>(eid));
                }
                if (std::find(types.begin(), types.end(), mctpTypePLDM) !=
                    types.end())
                {
                    info(
                        "Adding Endpoint networkId '{NETWORK}' and EID '{EID}'",
                        "NETWORK", networkId, "EID", eid);
                    mctpInfos.emplace_back(
                        MctpInfo(eid, emptyUUID, "", networkId));
                }
            }
        }
    }
}

void MctpDiscovery::addToExistingMctpInfos(const MctpInfos& addedInfos)
{
    for (const auto& mctpInfo : addedInfos)
    {
        if (std::find(existingMctpInfos.begin(), existingMctpInfos.end(),
                      mctpInfo) == existingMctpInfos.end())
        {
            existingMctpInfos.emplace_back(mctpInfo);
        }
    }
}

void MctpDiscovery::removeFromExistingMctpInfos(MctpInfos& mctpInfos,
                                                MctpInfos& removedInfos)
{
    for (const auto& mctpInfo : existingMctpInfos)
    {
        if (std::find(mctpInfos.begin(), mctpInfos.end(), mctpInfo) ==
            mctpInfos.end())
        {
            removedInfos.emplace_back(mctpInfo);
        }
    }
    for (const auto& mctpInfo : removedInfos)
    {
        info("Removing Endpoint networkId '{NETWORK}' and  EID '{EID}'",
             "NETWORK", std::get<3>(mctpInfo), "EID", std::get<0>(mctpInfo));
        existingMctpInfos.erase(std::remove(existingMctpInfos.begin(),
                                            existingMctpInfos.end(), mctpInfo),
                                existingMctpInfos.end());
    }
}

void MctpDiscovery::propertiesChangedCb(sdbusplus::message_t& msg)
{
    using Interface = std::string;
    using Property = std::string;
    using Value = std::string;
    using Properties = std::map<Property, std::variant<Value>>;

    Interface interface;
    Properties properties;

    msg.read(interface, properties);
    std::string objPath = msg.get_path();

    for (const auto& [key, valueVariant] : properties)
    {
        Value propVal = std::get<std::string>(valueVariant);
        auto availability = (propVal == "Available") ? true : false;

        if (key == MCTPConnectivityProp)
        {
            try
            {
                // Construct MctpInfo tuple
                auto service = pldm::utils::DBusHandler().getService(
                    objPath.c_str(), MCTPInterface);
                auto properties =
                    pldm::utils::DBusHandler().getDbusPropertiesVariant(
                        service.c_str(), objPath.c_str(), MCTPInterface);
                if (properties.contains("NetworkId") &&
                    properties.contains("EID") &&
                    properties.contains("SupportedMessageTypes"))
                {
                    auto networkId =
                        std::get<NetworkId>(properties.at("NetworkId"));
                    auto eid = std::get<mctp_eid_t>(properties.at("EID"));
                    auto types = std::get<std::vector<uint8_t>>(
                        properties.at("SupportedMessageTypes"));
                    MctpMedium mctpMediumStr = "";
                    if (eidToMctpMediums.find(eid) != eidToMctpMediums.end())
                    {
                        mctpMediumStr = eidToMctpMediums[eid];
                    }
                    if (std::find(types.begin(), types.end(), mctpTypePLDM) ==
                        types.end())
                    {
                        return;
                    }
                    MctpInfo mctpInfo(eid, emptyUUID, mctpMediumStr, networkId);
                    auto existingMatchIt = std::find(existingMctpInfos.begin(),
                                                     existingMctpInfos.end(),
                                                     mctpInfo);
                    if (existingMatchIt == existingMctpInfos.end() &&
                        availability)
                    {
                        // The endpoint not in existingMctpInfos and is
                        // available Add it to existingMctpInfos
                        info(
                            "Adding Endpoint networkId={NETWORK} EID={EID} by propertiesChanged signal",
                            "NETWORK", std::get<3>(mctpInfo), "EID",
                            unsigned(std::get<0>(mctpInfo)));
                        addToExistingMctpInfos(MctpInfos(1, mctpInfo));
                        handleMctpEndpoints(MctpInfos(1, mctpInfo));
                    }
                    else if (existingMatchIt != existingMctpInfos.end())
                    {
                        // The endpoint already in existingMctpInfos
                        updateMctpEndpointAvailability(mctpInfo, availability);
                    }
                }
            }
            catch (const sdbusplus::exception_t& e)
            {
                error(
                    "Error reading MCTP Endpoint property, ERROR={ERROR} PATH={PATH}",
                    "ERROR", e.what(), "PATH", objPath.c_str());
                return;
            }
        }
    }
}

void MctpDiscovery::discoverEndpoints(sdbusplus::message_t& msg)
{
    MctpInfos addedInfos;
    getAddedMctpInfos(msg, addedInfos);
    addToExistingMctpInfos(addedInfos);
    handleMctpEndpoints(addedInfos);
}

void MctpDiscovery::removeEndpoints(sdbusplus::message_t&)
{
    MctpInfos mctpInfos;
    MctpInfos removedInfos;
    std::map<MctpInfo, Availability> currentMctpInfoMap;
    getMctpInfos(currentMctpInfoMap);
    for (const auto& mapIt : currentMctpInfoMap)
    {
        mctpInfos.push_back(mapIt.first);
    }
    removeFromExistingMctpInfos(mctpInfos, removedInfos);
    handleRemovedMctpEndpoints(removedInfos);
}

void MctpDiscovery::handleMctpEndpoints(const MctpInfos& mctpInfos)
{
    for (const auto& handler : handlers)
    {
        if (handler)
        {
            handler->handleMctpEndpoints(mctpInfos);
        }
    }
}

void MctpDiscovery::handleRemovedMctpEndpoints(const MctpInfos& mctpInfos)
{
    for (const auto& handler : handlers)
    {
        if (handler)
        {
            handler->handleRemovedMctpEndpoints(mctpInfos);
        }
    }
}

void MctpDiscovery::updateMctpEndpointAvailability(const MctpInfo& mctpInfo,
                                                   Availability availability)
{
    for (const auto& handler : handlers)
    {
        if (handler)
        {
            handler->updateMctpEndpointAvailability(mctpInfo, availability);
        }
    }
}

} // namespace pldm
