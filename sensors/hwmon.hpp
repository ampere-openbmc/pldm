#pragma once

#include "sensors/interface.hpp"

#include <string>
#include <tuple>

namespace pldm
{

namespace sensor
{

namespace type
{
static constexpr uint8_t ctemp = 2;
static constexpr uint8_t cfan = 19;
static constexpr uint8_t kevils = 4;
static constexpr uint8_t cvolt = 5;
static constexpr uint8_t ccurr = 6;
static constexpr uint8_t cenergy = 8;
static constexpr uint8_t cpower = 7;
static constexpr uint8_t ccount = 67;
static constexpr uint8_t coem = 255;
} // namespace type

static constexpr auto typeAttrMap = {
    // 1 - hwmon class
    // 2 - unit
    // 3 - sysfs scaling factor
    // 4 - namespace
    std::make_tuple(pldm::sensor::type::ctemp, ValueInterface::Unit::DegreesC,
                    -3, "temperature"),
    std::make_tuple(pldm::sensor::type::cfan, ValueInterface::Unit::RPMS, 0,
                    "fan_tach"),
    std::make_tuple(pldm::sensor::type::cvolt, ValueInterface::Unit::Volts, -3,
                    "voltage"),
    std::make_tuple(pldm::sensor::type::ccurr, ValueInterface::Unit::Amperes,
                    -3, "current"),
    std::make_tuple(pldm::sensor::type::cenergy, ValueInterface::Unit::Joules,
                    -6, "energy"),
    std::make_tuple(pldm::sensor::type::cpower, ValueInterface::Unit::Watts, -6,
                    "power"),
    /*
     * Temporary use ValueInterface::Unit::RPMS for count, oem unit type
     * The patch to add count unit type is submited
     * https://gerrit.openbmc.org/c/openbmc/phosphor-dbus-interfaces/+/52258
     * But can not get the approval from community. Need more discussion.
     * ValueInterface::Unit::RPMS will be replaced by
     * ValueInterface::Unit::count if the patch is approved
     */
    std::make_tuple(pldm::sensor::type::ccount, ValueInterface::Unit::RPMS, 0,
                    "count"),
    std::make_tuple(pldm::sensor::type::coem, ValueInterface::Unit::RPMS, 0,
                    "oem"),
};

inline auto getHwmonType(decltype(typeAttrMap)::const_reference attrs)
{
    return std::get<0>(attrs);
}

inline auto getUnit(decltype(typeAttrMap)::const_reference attrs)
{
    return std::get<1>(attrs);
}

inline auto getScale(decltype(typeAttrMap)::const_reference attrs)
{
    return std::get<2>(attrs);
}

inline auto getNamespace(decltype(typeAttrMap)::const_reference attrs)
{
    return std::get<3>(attrs);
}

using AttributeIterator = decltype(*typeAttrMap.begin());
using Attributes =
    std::remove_cv<std::remove_reference<AttributeIterator>::type>::type;

/** @brief Get Attribute tuple for the type
 *
 *  Given a type, it tries to find the corresponding tuple
 *
 *  @param[in] type the sensor type
 *  @param[in,out] A pointer to the Attribute tuple
 */
bool getAttributes(uint8_t type, Attributes& attributes);

} //  namespace sensor

} //  namespace pldm
