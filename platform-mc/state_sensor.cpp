#include "state_sensor.hpp"

#include "libpldm/platform.h"

#include "common/utils.hpp"
#include "requester/handler.hpp"

#include <phosphor-logging/lg2.hpp>

#include <limits>
#include <regex>

namespace pldm
{
namespace platform_mc
{

ComponentStateSensorBase::ComponentStateSensorBase(
    const std::string& name, const std::string& invPath,
    const std::string& sensorPath, const bool& sensorDisabled,
    const uint16_t& entityType, const uint16_t& entityInstance,
    const uint16_t& containerID, const StateSetId& stateSetId,
    const PossibleStates& possibleStates,
    StateSensorHandler& stateSensorHandler) :
    sensorName(name),
    sensorPath(sensorPath), stateSetId(stateSetId),
    possibleStates(possibleStates), stateSensorHandler(stateSensorHandler)
{
    auto& bus = pldm::utils::DBusHandler::getBus();

    try
    {
        entityIntf = std::make_unique<EntityIntf>(bus, invPath.c_str());
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Failed to create Entity interface for path {PATH} error - {ERROR}",
            "PATH", invPath, "ERROR", e);
        throw sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument();
    }

    entityIntf->entityType(entityType);
    entityIntf->entityInstanceNumber(entityInstance);
    entityIntf->containerID(containerID);

    try
    {
        availabilityIntf =
            std::make_unique<AvailabilityIntf>(bus, sensorPath.c_str());
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Failed to create availability interface for state sensor {PATH} error - {ERROR}",
            "PATH", sensorPath, "ERROR", e);
        throw sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument();
    }
    availabilityIntf->available(true);

    try
    {
        operationalStatusIntf =
            std::make_unique<OperationalStatusIntf>(bus, sensorPath.c_str());
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Failed to create operational status interface for state sensor {PATH} error - {ERROR}",
            "PATH", sensorPath, "ERROR", e);
        throw sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument();
    }
    operationalStatusIntf->functional(!sensorDisabled);
}

int ComponentStateSensorBase::processOpState(const uint8_t& operationalState)
{
    if (!operationalStatusIntf || !availabilityIntf)
    {
        lg2::error(
            "Failed to process operational state {NAME} D-Bus interface don't exist.",
            "NAME", sensorName);
    }
    bool functional = false, available = false;
    switch (operationalState)
    {
        case PLDM_SENSOR_ENABLED:
            functional = true;
            available = true;
            break;
        case PLDM_SENSOR_DISABLED:
            functional = false;
            available = true;
            break;
        case PLDM_SENSOR_FAILED:
            functional = true;
            available = false;
            break;
        case PLDM_SENSOR_UNAVAILABLE:
        default:
            break;
    }
    auto rfunctional = operationalStatusIntf->functional(functional);
    auto ravailable = availabilityIntf->available(available);

    if (rfunctional != functional || ravailable != available)
    {
        return PLDM_ERROR;
    }
    return PLDM_SUCCESS;
}

template <typename TInf, typename TState>
int ComponentStateSensor<TInf, TState>::processSensorState(
    const uint8_t& sensorState)
{
    if (!operationalStatusIntf || !availabilityIntf)
    {
        lg2::error(
            "Failed to process sensor state {NAME} D-Bus interface don't exist.",
            "NAME", sensorName);
    }

    bool functional = operationalStatusIntf->functional();
    bool available = availabilityIntf->available();

    if (functional && available)
    {
        if (possibleStates.find(sensorState) == possibleStates.end())
        {
            lg2::error("Invalid state sensor state");
            return PLDM_ERROR_INVALID_DATA;
        }
        std::string propName;
        utils::PropertyValue propVal;
        auto rc = stateSensorHandler.getStateInfo(stateSetId, sensorState,
                                                  propName, propVal);
        if (rc)
        {
            return rc;
        }

        typename TInf::PropertiesVariant stateVar;
        // Convert string to PDI's enum when property type is not of D-Bus types
        // Currently, only checking against string and bool as interger and
        // double are assumed to not appear.
        if constexpr (!std::is_same_v<std::string, TState> &&
                      !std::is_same_v<bool, TState>)
        {
            stateVar = sdbusplus::message::convert_from_string<TState>(
                           std::get<std::string>(propVal))
                           .value();
        }
        else
        {
            auto value = std::get<TState>(propVal);
            stateVar = value;
        }
        stateSensorIntf->setPropertyByName(propName, stateVar);
    }
    return PLDM_ERROR;
}

StateSensor::StateSensor(const pldm_tid_t tid, const bool sensorDisabled,
                         const std::shared_ptr<PDR> pdr,
                         const std::vector<std::string>& sensorNames,
                         const std::string& associationPath,
                         StateSensorHandler& stateSensorHandler) :
    tid(tid),
    isPriority(false), stateSensorHandler(stateSensorHandler)
{
    auto pdrData = reinterpret_cast<const pldm_state_sensor_pdr*>(pdr->data());

    EntityType entityType = std::numeric_limits<uint16_t>::quiet_NaN();
    EntityInstance entityInstance = std::numeric_limits<uint16_t>::quiet_NaN();
    ContainerID containerID = std::numeric_limits<uint16_t>::quiet_NaN();
    CompositeStates compositeStates;

    sensorId = pdrData->sensor_id;
    entityType = pdrData->entity_type;
    entityInstance = pdrData->entity_instance;
    containerID = pdrData->container_id;
    compositeCount = pdrData->composite_sensor_count;

    // Parse PDR
    // using CompositeStates = std::vector<std::tuple<StateSetId,
    // PossibleStates>>;
    CompositeStates& sensors = compositeStates;
    auto statesPtr = pdrData->possible_states;

    while (compositeCount--)
    {
        auto state =
            reinterpret_cast<const state_sensor_possible_states*>(statesPtr);
        PossibleStates
            possibleStates{}; // using PossibleStates = std::set<uint8_t>;
        uint8_t possibleStatesPos{};
        auto updateStates = [&possibleStates,
                             &possibleStatesPos](const bitfield8_t& val) {
            for (int i = 0; i < CHAR_BIT; i++) // CHAR_BIT=8
            {
                if (val.byte & (1 << i))
                {
                    possibleStates.insert(possibleStatesPos * CHAR_BIT + i);
                }
            }
            possibleStatesPos++;
        };
        std::for_each(&state->states[0],
                      &state->states[state->possible_states_size],
                      updateStates);

        sensors.emplace_back(
            std::make_tuple(state->state_set_id, std::move(possibleStates)));

        if (compositeCount)
        {
            statesPtr += sizeof(state_sensor_possible_states) +
                         state->possible_states_size - 1;
        }
    }

    if (sizeof(compositeStates) != sizeof(sensorNames))
    {
        throw std::runtime_error("Invalid sensorNames size");
    }

    std::string sensorPath;
    std::string invPath;

    // filter out the physical/logical entity type encoded in the first bit
    entityType = entityType & ~(0x8000);
    entityInstance = entityInstance;
    containerID = containerID;

    // Create component sensor instances
    uint8_t sensorOffset = 0;
    for (const auto& name : sensorNames)
    {
        sensorPath = sensorNameSpace + name;
        sensorPath = std::regex_replace(sensorPath,
                                        std::regex("[^a-zA-Z0-9_/]+"), "_");
        invPath = associationPath + "/" + name;
        invPath = std::regex_replace(invPath, std::regex("[^a-zA-Z0-9_/]+"),
                                     "_");

        const auto& stateSetId = std::get<0>(compositeStates[sensorOffset]);
        const auto& possibleStates = std::get<1>(compositeStates[sensorOffset]);

        const auto& stateSetDbusInfo =
            stateSensorHandler.getStateSetInfo(stateSetId);

        auto interfaceStr = std::get<0>(stateSetDbusInfo).interface;
        std::unique_ptr<ComponentStateSensorBase> componentSensor;

        if (interfaceStr == "xyz.openbmc_project.State.Boot.Progress")
        {
            componentSensor = std::make_unique<ComponentStateSensor<
                BootProgressIntf, BootProgressIntf::ProgressStages>>(
                name, invPath, sensorPath, sensorDisabled, entityType,
                entityInstance, containerID, stateSetId, possibleStates,
                stateSensorHandler);
        }
        else if (interfaceStr ==
                 "xyz.openbmc_project.Control.Power.ACPIPowerState")
        {
            componentSensor = std::make_unique<
                ComponentStateSensor<ACPIIntf, ACPIIntf::ACPI>>(
                name, invPath, sensorPath, sensorDisabled, entityType,
                entityInstance, containerID, stateSetId, possibleStates,
                stateSensorHandler);
        }
        else
        {
            lg2::error("Unsupported sensor state D-Bus interface string, "
                       "{INTERFACE}.",
                       "INTERFACE", interfaceStr);
        }

        componentSensors.emplace_back(std::move(componentSensor));

        timeStamp = 0;

        sensorOffset++;
    }
}

int StateSensor::processStateSensorReadings(
    const std::array<get_sensor_state_field, 8>& stateField,
    const uint8_t& comp_sensor_count)
{
    uint8_t operationalState;
    uint8_t presentState;
    // uint8_t previousState;

    for (uint8_t sensorOffset = 0; sensorOffset < comp_sensor_count;
         sensorOffset++)
    {
        operationalState = stateField[sensorOffset].sensor_op_state;
        presentState = stateField[sensorOffset].present_state;
        // previousState = stateField[sensorOffset].previous_state;

        // emitStateSensorEventSignal(tid, sensorId, sensorOffset,
        //  eventState,
        //  previousEventState);

        if (sensorOffset >= compositeCount)
        {
            lg2::error("Invalid state sensor offset in state sensor reading, "
                       "tid={TID}, sensorId={ID}, sensorOffset={OFFSET}.",
                       "TID", tid, "ID", sensorId, "OFFSET", sensorOffset);
            return PLDM_ERROR_INVALID_DATA;
        }

        auto rc = processOpState(operationalState, sensorOffset);
        rc = processSensorState(presentState, sensorOffset);
        if (rc)
        {
            lg2::error(
                "Failed to proccess StateSensorReadings, "
                "tid={TID}, sensorId={ID}, sensorOffset={OFFSET}, rc={RC}.",
                "TID", tid, "ID", sensorId, "OFFSET", sensorOffset, "RC", rc);
        }
    }
    return PLDM_SUCCESS;
}

} // namespace platform_mc
} // namespace pldm
