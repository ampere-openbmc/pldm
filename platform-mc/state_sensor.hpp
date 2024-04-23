#pragma once

#include "libpldm/platform.h"
#include "libpldm/pldm.h"

#include "common/types.hpp"
#include "platform-mc/state_dbus_parser.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/server/object.hpp>
#include <xyz/openbmc_project/Association/Definitions/server.hpp>
#include <xyz/openbmc_project/Control/Power/ACPIPowerState/server.hpp>
#include <xyz/openbmc_project/Inventory/Source/PLDM/Entity/server.hpp>
#include <xyz/openbmc_project/State/Boot/Progress/server.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/server.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/server.hpp>

namespace pldm
{
namespace platform_mc
{
using namespace pldm::pdr;

using OperationalStatusIntf =
    sdbusplus::server::object_t<sdbusplus::xyz::openbmc_project::State::
                                    Decorator::server::OperationalStatus>;
using AvailabilityIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::State::Decorator::server::Availability>;
using AssociationIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::Association::server::Definitions>;
using EntityIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::Inventory::Source::PLDM::server::Entity>;
using ACPIIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::Control::Power::server::ACPIPowerState>;
using BootProgressIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::State::Boot::server::Progress>;

/**
 * @brief ComponentStateSensorBase
 *
 * This class represents a base component sensor of a state sensor, processes
 * state or operational reading and publishes results to D-Bus.
 */
class ComponentStateSensorBase
{
  public:
    ComponentStateSensorBase(
        const std::string& name, const std::string& invPath,
        const std::string& sensorPath, const bool& sensorDisabled,
        const uint16_t& entityType, const uint16_t& entityInstance,
        const uint16_t& containerID, const StateSetId& stateSetId,
        const PossibleStates& possibleStates,
        StateSensorHandler& stateSensorHandler);

    virtual ~ComponentStateSensorBase() = default;

    /** @brief Process the Operational reading of the sensor and write to D-Bus
     */
    int processOpState(const uint8_t& operationalState);

    /** @brief Process the state reading of the sensor and write to D-Bus */
    virtual int processSensorState(const uint8_t& sensorState) = 0;

    inline std::string getSensorName()
    {
        return sensorName;
    }

  protected:
    /** @brief Sensor name */
    std::string sensorName;
    /** @brief Sensor D-Bus object path */
    std::string sensorPath;
    /** @brief State.Decorator.Availability D-Bus interface */
    std::unique_ptr<AvailabilityIntf> availabilityIntf = nullptr;
    /** @brief State.Decorator.OperationalStatus D-Bus interface */
    std::unique_ptr<OperationalStatusIntf> operationalStatusIntf = nullptr;
    /** @brief Inventory.Source.PLDM.Entity D-Bus interface */
    std::unique_ptr<EntityIntf> entityIntf = nullptr;
    /** @brief Sensor state set ID from PDR */
    const StateSetId stateSetId;
    /** @brief Sensor possible states from PDR as a vector of bytes */
    const PossibleStates possibleStates;
    /** @brief The handler to hold state sensor JSON configs */
    StateSensorHandler& stateSensorHandler;
};

/**
 * @brief ComponentStateSensor
 *
 * This class represents a component sensor of a state sensor which inherits
 * the ComponentStateSensorBase class and has its separate template member to
 * hold the D-Bus interface that is reponsible for the state set ID of a
 * specific component sensor
 */
template <typename TInf, typename TState>
class ComponentStateSensor : public ComponentStateSensorBase
{
  public:
    ComponentStateSensor(const std::string& name, const std::string& invPath,
                         const std::string& sensorPath,
                         const bool& sensorDisabled, const uint16_t& entityType,
                         const uint16_t& entityInstance,
                         const uint16_t& containerID,
                         const StateSetId& stateSetId,
                         const PossibleStates& possibleStates,
                         StateSensorHandler& stateSensorHandler) :
        ComponentStateSensorBase(name, invPath, sensorPath, sensorDisabled,
                                 entityType, entityInstance, containerID,
                                 stateSetId, possibleStates, stateSensorHandler)
    {
        try
        {
            stateSensorIntf = std::make_unique<TInf>(
                pldm::utils::DBusHandler::getBus(), sensorPath.c_str());
        }
        catch (const sdbusplus::exception_t& e)
        {
            lg2::error(
                "Failed to create state sensor interface for sensor {PATH} error - {ERROR}",
                "PATH", sensorPath, "ERROR", e);
            throw sdbusplus::xyz::openbmc_project::Common::Error::
                InvalidArgument();
        }
    }
    ~ComponentStateSensor() = default;

    /** @brief Process the state reading of the sensor and write to D-Bus */
    int processSensorState(const uint8_t& sensorState) override;

  private:
    /** @brief The D-Bus interface that is associated with the state set ID
     * of this sensor, and can be decided from the state sensor JSON config*/
    std::unique_ptr<TInf> stateSensorIntf = nullptr;
};

/**
 * @brief StateSensor
 *
 * This class represent a state sensor (can be a composite sensor),
 * manages component sensors, handles readings and forwards results
 * to the corresponding component sensor
 */
class StateSensor
{
  public:
    StateSensor(const pldm_tid_t tid, const bool sensorDisabled,
                const std::shared_ptr<PDR> pdr,
                const std::vector<std::string>& sensorNames,
                const std::string& associationPath,
                StateSensorHandler& stateSensorHandler);

    ~StateSensor(){};

    /** @brief Forward the operational reading to the component sensor */
    inline int processOpState(const uint8_t& operationalState,
                              const uint8_t& sensorOffset)
    {
        return componentSensors[sensorOffset]->processOpState(operationalState);
    }
    /** @brief Forward the state reading to the component sensor */
    inline int processSensorState(const uint8_t& sensorState,
                                  const uint8_t& sensorOffset)
    {
        return componentSensors[sensorOffset]->processSensorState(sensorState);
    }
    /** @brief Process the state sensor reading from GetStateSensorReading
     * response */
    int processStateSensorReadings(
        const std::array<get_sensor_state_field, 8>& stateField,
        const uint8_t& comp_sensor_count);

    /** @brief Terminus ID which the sensor belongs to */
    pldm_tid_t tid;

    /** @brief Sensor ID */
    uint16_t sensorId;

    /** @brief Sensor composite count */
    CompositeCount compositeCount;

    /** @brief  The time stamp since last getSensorReading command in usec */
    uint64_t timeStamp;

    /** @brief  sensorNameSpace */
    std::string sensorNameSpace = "/xyz/openbmc_project/sensors/state/";

    /** @brief indicate if sensor is polled in priority */
    bool isPriority;

  private:
    /** @brief List of pointers to component sensors*/
    std::vector<std::unique_ptr<ComponentStateSensorBase>> componentSensors;
    /** @brief The handler to parse D-Bus info configuration for each state
     * set*/
    StateSensorHandler& stateSensorHandler;
};
} // namespace platform_mc
} // namespace pldm
