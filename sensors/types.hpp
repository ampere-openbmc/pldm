#pragma once

#include "interface.hpp"

#include <any>
#include <chrono>
#include <map>
#include <string>
#include <tuple>
#include <utility>
namespace pldm
{

namespace sensor
{

using InterfaceMap = std::map<InterfaceType, std::any>;
using ObjectInfo = std::tuple<sdbusplus::bus::bus*, std::string, InterfaceMap>;
using ObjectStateData = std::pair<std::string, ObjectInfo>;

} // namespace sensor

} // namespace pldm