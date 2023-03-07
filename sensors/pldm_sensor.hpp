#pragma once

#include "common/types.hpp"
#include "common/utils.hpp"
#include "libpldmresponder/event_parser.hpp"
#include "libpldmresponder/pdr_utils.hpp"
#include "sensors/interface.hpp"
#include "sensors/thresholds.hpp"

#include <memory>
#include <optional>
#include <unordered_set>

namespace pldm
{

namespace sensor
{

using namespace pldm::sensor;

/** @class PldmSensor
 *  @brief Sensor object
 *  @details Sensor object to create and modify an associated device's sensor
 *  attributes based on the key type of each sensor in the set provided by the
 *  device.
 */
class PldmSensor
{
  public:
    /**
     * @brief Constructs PldmSensor object
     *
     * @param[in] bus - Root D-Bus interface
     * @param[in] name - Sensor name
     * @param[in] baseUnit - Sensor base unit
     * @param[in] unitModifier - Sensor unit modifier
     * @param[in] offset - Sensor unit offset
     * @param[in] resolution - Sensor unit resolution
     * @param[in] warningHigh - Sensor warning high threshold
     * @param[in] warningLow - Sensor warning low threshold
     * @param[in] criticalHigh - Sensor critical high threshold
     * @param[in] criticalLow - Sensor critical low threshold
     */
    PldmSensor(sdbusplus::bus::bus& bus, const std::string& name,
               uint8_t baseUnit, int8_t unitModifier, double offset,
               double resolution, double warningHigh, double warningLow,
               double criticalHigh, double criticalLow);

    /**
     * @brief De-constructs PldmSensor object
     */
    ~PldmSensor();

    /**
     * @brief Create sensor D-Bus interfaces
     * @details After init the sensor data, call createSensor to create the
     * sensor interfaces such as value, functional status, thresholds
     *
     * @return - Shared pointer to the object data
     */
    std::optional<ObjectStateData> createSensor();

    /**
     * @brief Add value interface and value property for sensor
     * @details When a sensor has an associated input file, the Sensor.Value
     * interface is added along with setting the Value property to the
     * corresponding value found in the input file.
     *
     * @param[in] info - Sensor object information
     * @param[in] value - Sensor value
     * @param[in] max - Sensor max
     * @param[in] min - Sensor min
     *
     * @return - Shared pointer to the value object
     */
    std::shared_ptr<ValueObject> addValueInterface(ObjectInfo& info,
                                                   SensorValueType value,
                                                   SensorValueType max,
                                                   SensorValueType min);

    /**
     * @brief Add status interface and functional property for sensor
     * @details OperationalStatus interface is added and the Functional property
     * is set depending on whether a fault file exists and if it does it will
     * also depend on the content of the fault file. _hasFaultFile will also be
     * set to true if fault file exists.
     *
     * @param[in] info - Sensor object information
     * @param[in] functional - Functional status
     *
     * @return - Shared pointer to the status object
     */
    std::shared_ptr<StatusObject> addStatusInterface(ObjectInfo& info,
                                                     bool functional);

    /**
     * @brief Apply unitModifier to sensor value
     * @details Use unitModifier to modify the raw value from the PLDM
     * interface.
     *
     * @param[in] value - Sensor value
     *
     * @return - The modified value in type SensorValueType
     */
    SensorValueType adjustValue(SensorValueType value);

    /**
     * @brief Update sensor value
     * @details Update sensor value base on the return value from PLDM
     *
     * @param[in] sensorValue - Sensor value
     *
     * @return - none
     */
    void updateValue(SensorValueType sensorValue);

    /**
     * @brief Set sensor functional status
     *
     * @param[in] functional - functional status
     *
     * @return - none
     */
    void setFunctionalStatus(bool functional)
    {
        statusInterface->functional(functional);
    }

    /**
     * @brief Get sensor functional status
     *
     * @return - functional status
     */
    bool getFunctionalStatus()
    {
        return statusInterface->functional();
    }

    /**
     * @brief Get sensor path
     *
     * @return - sensor path
     */
    std::string getSensorPath()
    {
        return sensorPath;
    }

    void initMinMaxValue(double minValue, double maxValue)
    {
        sensorMinValue = minValue;
        sensorMaxValue = maxValue;

        return;
    }

  private:
    std::string _root = "/xyz/openbmc_project/sensors";
    /** @brief reference of main D-bus interface of pldmd devices */
    sdbusplus::bus::bus& _bus;
    /** @brief Sensor data fields */
    std::string sensorName;
    std::string sensorPath;
    uint8_t baseUnit;
    int8_t unitModifier;
    double offset;
    double resolution;
    double warningHigh;
    double warningLow;
    double criticalHigh;
    double criticalLow;
    double sensorMaxValue = std::numeric_limits<double>::quiet_NaN();
    double sensorMinValue = std::numeric_limits<double>::quiet_NaN();
    /** @brief Store value interface */
    std::shared_ptr<ValueObject> valueInterface;
    /** @brief Store functional status interface */
    std::shared_ptr<StatusObject> statusInterface;
    /** @brief Store warning thresholds interface */
    std::shared_ptr<WarningObject> warnObject;
    /** @brief Store critical thresholds interface */
    std::shared_ptr<CriticalObject> critObject;
    SensorValueType lastValue = std::numeric_limits<double>::quiet_NaN();
};

} // namespace sensor

} // namespace pldm
