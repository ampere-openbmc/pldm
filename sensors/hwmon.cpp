#include "sensors/hwmon.hpp"

namespace pldm
{

namespace sensor
{

bool getAttributes(uint8_t type, Attributes& attributes)
{
    auto a =
        std::find_if(typeAttrMap.begin(), typeAttrMap.end(),
                     [&](const auto& e) { return type == getHwmonType(e); });

    if (a == typeAttrMap.end())
    {
        return false;
    }

    attributes = *a;
    return true;
}

} //  namespace sensor

} //  namespace pldm
