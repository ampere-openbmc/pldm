
#pragma once

#include <sdbusplus/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/server.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/server.hpp>

namespace pldm
{

namespace sensor
{

template <typename... T>
using ServerObject = typename sdbusplus::server::object::object<T...>;

using ValueInterface = sdbusplus::xyz::openbmc_project::Sensor::server::Value;
using ValueObject = ServerObject<ValueInterface>;
using WarningInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Warning;
using WarningObject = ServerObject<WarningInterface>;
using CriticalInterface =
    sdbusplus::xyz::openbmc_project::Sensor::Threshold::server::Critical;
using CriticalObject = ServerObject<CriticalInterface>;
using StatusInterface = sdbusplus::xyz::openbmc_project::State::Decorator::
    server::OperationalStatus;
using StatusObject = ServerObject<StatusInterface>;

using SensorValueType = double;

enum class InterfaceType
{
    VALUE,
    WARN,
    CRIT,
    STATUS,
};

} // namespace sensor

} // namespace pldm