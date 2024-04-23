#include "state_dbus_parser.hpp"

#include "event_parser.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <filesystem>
#include <fstream>
#include <set>

PHOSPHOR_LOG2_USING;

namespace pldm::platform_mc
{
namespace fs = std::filesystem;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

const Json emptyJson{};
const std::vector<Json> emptyJsonList{};

const std::set<std::string_view> supportedDbusPropertyTypes = {
    "bool",     "uint8_t", "int16_t",  "uint16_t", "int32_t",
    "uint32_t", "int64_t", "uint64_t", "double",   "string"};

StateSensorHandler::StateSensorHandler(const std::string& dirPath)
{
    fs::path dir(dirPath);
    if (!fs::exists(dir) || fs::is_empty(dir))
    {
        lg2::error(
            "Event config directory does not exist or empty at path {DIR_PATH}",
            "DIR_PATH", dirPath.c_str());
        return;
    }

    for (auto& file : fs::directory_iterator(dirPath))
    {
        std::ifstream jsonFile(file.path());

        auto data = Json::parse(jsonFile, nullptr, false);
        if (data.is_discarded())
        {
            lg2::error(
                "Parsing Event state sensor JSON file failed for file {FILE_PATH}",
                "FILE_PATH", file.path().c_str());
            continue;
        }

        auto entries = data.value("entries", emptyJsonList);
        for (const auto& entry : entries)
        {
            StateSetId stateSetid =
                static_cast<uint16_t>(entry.value("stateSetId", 0));

            pldm::utils::DBusMapping dbusInfo{};

            auto dbus = entry.value("dbus", emptyJson);
            dbusInfo.objectPath = "";
            dbusInfo.interface = dbus.value("interface", "");
            dbusInfo.propertyName = dbus.value("property_name", "");
            dbusInfo.propertyType = dbus.value("property_type", "");
            if (dbusInfo.interface.empty() || dbusInfo.propertyName.empty() ||
                !supportedDbusPropertyTypes.contains(dbusInfo.propertyType))
            {
                lg2::error(
                    "Invalid D-Bus configuration at path {OBJ_PATH}, interface {INTF}, property {PROP}, type {TYPE}",
                    "OBJ_PATH", dbusInfo.objectPath.c_str(), "INTF",
                    dbusInfo.interface, "PROP", dbusInfo.propertyName, "TYPE",
                    dbusInfo.propertyType);
                continue;
            }

            auto sensorStates = entry.value("event_states", emptyJsonList);
            auto propertyValues = dbus.value("property_values", emptyJsonList);
            if ((sensorStates.size() == 0) || (propertyValues.size() == 0) ||
                (sensorStates.size() != propertyValues.size()))
            {
                lg2::error(
                    "Invalid event state JSON configuration with event state size {STATE_SIZE}, property value type {VAL_SIZE}",
                    "STATE_SIZE", sensorStates.size(), "VAL_SIZE",
                    propertyValues.size());
                continue;
            }

            auto stateToDbusMap = mapStateToDBusVal(
                sensorStates, propertyValues, dbusInfo.propertyType);
            stateSetMap.emplace(stateSetid,
                                std::make_tuple(std::move(dbusInfo),
                                                std::move(stateToDbusMap)));
        }
    }
}

StateToDBusValue StateSensorHandler::mapStateToDBusVal(
    const Json& sensorStates, const Json& propertyValues, std::string_view type)
{
    StateToDBusValue stateToDbusMap{};
    auto stateIt = sensorStates.begin();
    auto propIt = propertyValues.begin();

    for (; stateIt != sensorStates.end(); ++stateIt, ++propIt)
    {
        auto propValue = utils::jsonEntryToDbusVal(type, propIt.value());
        stateToDbusMap.emplace((*stateIt).get<uint8_t>(), std::move(propValue));
    }

    return stateToDbusMap;
}

int StateSensorHandler::getStateInfo(const StateSetId& entry,
                                     const pdr::EventState& state,
                                     std::string& propName,
                                     utils::PropertyValue& propVal)
{
    try
    {
        auto& [dbusMapping, stateToDbusMap] = stateSetMap.at(entry);
        try
        {
            propVal = stateToDbusMap.at(state);
            propName = dbusMapping.propertyName;
            return PLDM_SUCCESS;
        }
        catch (const std::out_of_range& e)
        {
            lg2::error("Invalid event state {EVENT_STATE}: {ERROR}",
                       "EVENT_STATE", state, "ERROR", e);
            return PLDM_ERROR_INVALID_DATA;
        }
    }
    catch (const std::out_of_range& e)
    {
        lg2::error(
            "D-Bus info for state set id {ENTRY} does not exists: {ERROR}",
            "ENTRY", entry, "ERROR", e);
        return PLDM_ERROR_INVALID_DATA;
    }
}

} // namespace pldm::platform_mc
