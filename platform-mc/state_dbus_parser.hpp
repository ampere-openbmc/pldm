#pragma once

#include "common/types.hpp"
#include "common/utils.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace pldm::platform_mc
{
using Json = nlohmann::json;
using namespace pldm::pdr;
using namespace pldm::utils;
using StateToDBusValue = std::map<pdr::EventState, pldm::utils::PropertyValue>;
using StateSetDBusInfo = std::tuple<pldm::utils::DBusMapping, StateToDBusValue>;
using StateSetMap = std::map<StateSetId, StateSetDBusInfo>;

/** @class StateSensorHandler
 *
 *  @brief Parses the event state sensor configuration JSON file and build
 *         the lookup data structure, which can map the event state for a
 *         sensor in the PlatformEventMessage command to a D-Bus property and
 *         the property value.
 */
//[TODO ChauLy May change class name to StateSensorParser]
class StateSensorHandler
{
  public:
    StateSensorHandler() = delete;

    /** @brief Parse the event state sensor configuration JSON file and build
     *         the lookup data stucture.
     *
     *  @param[in] dirPath - directory path which has the config JSONs
     */
    explicit StateSensorHandler(const std::string& dirPath);
    virtual ~StateSensorHandler() = default;
    StateSensorHandler(const StateSensorHandler&) = default;
    StateSensorHandler& operator=(const StateSensorHandler&) = default;
    StateSensorHandler(StateSensorHandler&&) = default;
    StateSensorHandler& operator=(StateSensorHandler&&) = default;

    /** @brief Helper API to get D-Bus information for a state of a state set
     *
     *  @param[in] entry - state set id
     *  @param[in] state - event state
     *  @param[out] propName - D-Bus property name
     *  @param[out] propVal - D-Bus property value
     *
     *  @return PLDM completion code
     */
    int getStateInfo(const StateSetId& entry, const pdr::EventState& state,
                     std::string& propName, utils::PropertyValue& propVal);

    /** @brief Helper API to get D-Bus information for a state set
     *
     *  @param[in] entry - state set id
     *
     *  @return D-Bus information corresponding to the SensorEntry
     */
    const StateSetDBusInfo& getStateSetInfo(const StateSetId& entry) const
    {
        return stateSetMap.at(entry);
    }

  private:
    StateSetMap stateSetMap; //!< a map of StateSensorEntry to D-Bus information

    /** @brief Create a map of EventState to D-Bus property values from
     *         the information provided in the event state configuration
     *         JSON
     *
     *  @param[in] eventStates - a JSON array of event states
     *  @param[in] propertyValues - a JSON array of D-Bus property values
     *  @param[in] type - the type of D-Bus property
     *
     *  @return a map of EventState to D-Bus property values
     */
    StateToDBusValue mapStateToDBusVal(const Json& eventStates,
                                       const Json& propertyValues,
                                       std::string_view type);
};

} // namespace pldm::platform_mc
