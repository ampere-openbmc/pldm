#include "sensors/pldm_sensor.hpp"

#include "common/utils.hpp"
#include "sensors/hwmon.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace pldm
{

namespace sensor
{

using namespace pldm::sensor;
using namespace pldm::utils;

// Initialization for Warning Objects
decltype(Thresholds<WarningObject>::setLo) Thresholds<WarningObject>::setLo =
    &WarningObject::warningLow;
decltype(Thresholds<WarningObject>::setHi) Thresholds<WarningObject>::setHi =
    &WarningObject::warningHigh;
decltype(Thresholds<WarningObject>::getLo) Thresholds<WarningObject>::getLo =
    &WarningObject::warningLow;
decltype(Thresholds<WarningObject>::getHi) Thresholds<WarningObject>::getHi =
    &WarningObject::warningHigh;
decltype(Thresholds<WarningObject>::alarmLo)
    Thresholds<WarningObject>::alarmLo = &WarningObject::warningAlarmLow;
decltype(Thresholds<WarningObject>::alarmHi)
    Thresholds<WarningObject>::alarmHi = &WarningObject::warningAlarmHigh;
decltype(Thresholds<WarningObject>::getAlarmLow)
    Thresholds<WarningObject>::getAlarmLow = &WarningObject::warningAlarmLow;
decltype(Thresholds<WarningObject>::getAlarmHigh)
    Thresholds<WarningObject>::getAlarmHigh = &WarningObject::warningAlarmHigh;
decltype(Thresholds<WarningObject>::assertLowSignal)
    Thresholds<WarningObject>::assertLowSignal =
        &WarningObject::warningLowAlarmAsserted;
decltype(Thresholds<WarningObject>::assertHighSignal)
    Thresholds<WarningObject>::assertHighSignal =
        &WarningObject::warningHighAlarmAsserted;
decltype(Thresholds<WarningObject>::deassertLowSignal)
    Thresholds<WarningObject>::deassertLowSignal =
        &WarningObject::warningLowAlarmDeasserted;
decltype(Thresholds<WarningObject>::deassertHighSignal)
    Thresholds<WarningObject>::deassertHighSignal =
        &WarningObject::warningHighAlarmDeasserted;

// Initialization for Critical Objects
decltype(Thresholds<CriticalObject>::setLo) Thresholds<CriticalObject>::setLo =
    &CriticalObject::criticalLow;
decltype(Thresholds<CriticalObject>::setHi) Thresholds<CriticalObject>::setHi =
    &CriticalObject::criticalHigh;
decltype(Thresholds<CriticalObject>::getLo) Thresholds<CriticalObject>::getLo =
    &CriticalObject::criticalLow;
decltype(Thresholds<CriticalObject>::getHi) Thresholds<CriticalObject>::getHi =
    &CriticalObject::criticalHigh;
decltype(Thresholds<CriticalObject>::alarmLo)
    Thresholds<CriticalObject>::alarmLo = &CriticalObject::criticalAlarmLow;
decltype(Thresholds<CriticalObject>::alarmHi)
    Thresholds<CriticalObject>::alarmHi = &CriticalObject::criticalAlarmHigh;
decltype(Thresholds<CriticalObject>::getAlarmLow)
    Thresholds<CriticalObject>::getAlarmLow = &CriticalObject::criticalAlarmLow;
decltype(Thresholds<CriticalObject>::getAlarmHigh)
    Thresholds<CriticalObject>::getAlarmHigh =
        &CriticalObject::criticalAlarmHigh;
decltype(Thresholds<CriticalObject>::assertLowSignal)
    Thresholds<CriticalObject>::assertLowSignal =
        &CriticalObject::criticalLowAlarmAsserted;
decltype(Thresholds<CriticalObject>::assertHighSignal)
    Thresholds<CriticalObject>::assertHighSignal =
        &CriticalObject::criticalHighAlarmAsserted;
decltype(Thresholds<CriticalObject>::deassertLowSignal)
    Thresholds<CriticalObject>::deassertLowSignal =
        &CriticalObject::criticalLowAlarmDeasserted;
decltype(Thresholds<CriticalObject>::deassertHighSignal)
    Thresholds<CriticalObject>::deassertHighSignal =
        &CriticalObject::criticalHighAlarmDeasserted;

/**
 * @brief Constructs PldmSensor object
 */
PldmSensor::PldmSensor(sdbusplus::bus::bus& bus, const std::string& name,
                       uint8_t baseUnit, int8_t unitModifier, double offset,
                       double resolution, double warningHigh, double warningLow,
                       double criticalHigh, double criticalLow) :
    _bus(bus),
    sensorName(name), baseUnit(baseUnit), unitModifier(unitModifier),
    offset(offset), resolution(resolution), warningHigh(warningHigh),
    warningLow(warningLow), criticalHigh(criticalHigh), criticalLow(criticalLow)
{}

/**
 * @brief De-constructs PldmSensor object
 */
PldmSensor::~PldmSensor()
{
    _bus.emit_object_removed(sensorPath.c_str());
}

/**
 * @brief Create sensor interfaces
 * @details After init the sensor data, call createSensor to create the
 * sensor interfaces such as value, functional status, thresholds
 *
 * @return - Shared pointer to the object data
 */
std::optional<ObjectStateData> PldmSensor::createSensor()
{
    Attributes attrs;
    if (!std::isnan(warningLow))
    {
        warningLow = adjustValue(warningLow);
    }
    if (!std::isnan(warningHigh))
    {
        warningHigh = adjustValue(warningHigh);
    }
    if (!std::isnan(criticalLow))
    {
        criticalLow = adjustValue(criticalLow);
    }
    if (!std::isnan(criticalHigh))
    {
        criticalHigh = adjustValue(criticalHigh);
    }
    auto type = getAttributes(baseUnit, attrs);
    if (!type)
    {
        std::cerr << "Failed to find sensor type: " << type
                  << " Use default type. sensorId " << sensorName << std::endl;
        return {};
    }

    sensorPath = _root + "/" + getNamespace(attrs) + "/" + sensorName;

    double sensorValue = std::numeric_limits<double>::quiet_NaN();
    double sensorMaxValue = std::numeric_limits<double>::quiet_NaN();
    double sensorMinValue = std::numeric_limits<double>::quiet_NaN();
    ObjectInfo info(&_bus, std::move(sensorPath), InterfaceMap());
    try
    {
        statusInterface = addStatusInterface(info, true);
        valueInterface = addValueInterface(info, sensorValue, sensorMaxValue,
                                           sensorMinValue);
        if (!valueInterface)
        {
            return {};
        }
        valueInterface->unit(getUnit(attrs));
    }
    catch (const std::system_error& e)
    {
        return {};
    }
    warnObject =
        addThreshold<WarningObject>(info, sensorValue, warningLow, warningHigh);
    critObject = addThreshold<CriticalObject>(info, sensorValue, criticalLow,
                                              criticalHigh);
    valueInterface->emit_object_added();

    return std::make_pair(sensorName, std::move(info));
}

/**
 * @brief Add value interface for Sensor
 */
std::shared_ptr<ValueObject>
    PldmSensor::addValueInterface(ObjectInfo& info, SensorValueType value,
                                  SensorValueType max, SensorValueType min)
{
    std::shared_ptr<ValueObject> iface = nullptr;
    auto& bus = *std::get<sdbusplus::bus::bus*>(info);
    auto& obj = std::get<InterfaceMap>(info);
    auto& objPath = std::get<std::string>(info);

    try
    {
        auto& statusIface = std::any_cast<std::shared_ptr<StatusObject>&>(
            obj[InterfaceType::STATUS]);
        assert(statusIface);

        iface = std::make_shared<ValueObject>(bus, objPath.c_str(),
                                              ValueObject::action::defer_emit);
        iface->value(value);
        iface->maxValue(max);
        iface->minValue(min);
        obj[InterfaceType::VALUE] = iface;
    }
    catch (const std::system_error& e)
    {
        return nullptr;
    }

    return iface;
}

/**
 * @brief Add functional status interface for Sensor
 */
std::shared_ptr<StatusObject> PldmSensor::addStatusInterface(ObjectInfo& info,
                                                             bool functional)
{
    std::shared_ptr<StatusObject> iface = nullptr;
    auto& objPath = std::get<std::string>(info);
    auto& obj = std::get<InterfaceMap>(info);
    auto& bus = *std::get<sdbusplus::bus::bus*>(info);

    try
    {
        iface = std::make_shared<StatusObject>(
            bus, objPath.c_str(), StatusObject::action::defer_emit);
        iface->functional(functional);
        obj[InterfaceType::STATUS] = iface;
    }
    catch (const std::system_error& e)
    {
        return nullptr;
    }

    return iface;
}

/**
 * @brief Apply unitModifier to sensor value
 */
SensorValueType PldmSensor::adjustValue(SensorValueType value)
{
    if (value < 0)
    {
        return value;
    }

    if constexpr (std::is_same<SensorValueType, double>::value)
    {
        value =
            (value * resolution + offset) * std::pow(10, signed(unitModifier));
    }

    return value;
}

/**
 * @brief Update sensor interfaces
 */
void PldmSensor::updateValue(SensorValueType sensorValue)
{
    auto value = adjustValue(sensorValue);

    if (value == lastValue)
    {
        return;
    }
    lastValue = value;
    valueInterface->value(lastValue);

    if (!std::isnan(lastValue))
    {
        if (warnObject)
        {
            checkThresholds<WarningObject>(warnObject, lastValue);
        }
        if (critObject)
        {
            checkThresholds<CriticalObject>(critObject, lastValue);
        }
    }

    return;
}

} // namespace sensor

} // namespace pldm