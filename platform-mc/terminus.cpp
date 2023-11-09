#include "terminus.hpp"

#include "libpldm/platform.h"

#include "terminus_manager.hpp"

namespace pldm
{
namespace platform_mc
{
/* default the max message buffer size BMC supported to 4K bytes */
#define MAX_MESSAGE_BUFFER_SIZE 4096

Terminus::Terminus(pldm_tid_t tid, uint64_t supportedTypes) :
    initialized(false), maxBufferSize(MAX_MESSAGE_BUFFER_SIZE),
    synchronyConfigurationSupported(0), pollEvent(false), tid(tid),
    supportedTypes(supportedTypes)
{}

bool Terminus::doesSupportType(uint8_t type)
{
    return supportedTypes.test(type);
}

bool Terminus::doesSupportCommand(uint8_t type, uint8_t command)
{
    if (!doesSupportType(type))
    {
        return false;
    }

    try
    {
        const size_t idx = type * (PLDM_MAX_CMDS_PER_TYPE / 8) + (command / 8);
        if (idx >= supportedCmds.size())
        {
            return false;
        }

        if (supportedCmds[idx] & (1 << (command % 8)))
        {
            lg2::info(
                "PLDM type {TYPE} command {CMD} is supported by terminus {TID}",
                "TYPE", type, "CMD", command, "TID", getTid());
            return true;
        }
    }
    catch (const std::exception& e)
    {
        return false;
    }

    return false;
}

std::string Terminus::findTerminusName()
{
    for (auto entityAuxiliaryNames : entityAuxiliaryNamesTbl)
    {
        const auto& [key, entityNames] = *entityAuxiliaryNames;
        /**
         * + containerId : 0x0000 = "SYSTEM". If this value is 0x0000, the
         * containing entity is considered to be the overall system.
         * + Only have one overall system entity in terminus
         */
        if (key.containerId == PLDM_OVERALL_SYTEM_CONTAINER_ID &&
            key.instanceIdx == 1)
        {
            if (entityNames.size() == 0)
            {
                return "";
            }
            return entityNames[0].second;
        }
    }

    return "";
}

bool Terminus::createInventoryPath(std::string tName)
{
    if (tName == "")
    {
        return false;
    }

    inventoryPath = "/xyz/openbmc_project/inventory/Item/Board/" + tName;
    try
    {
        inventoryItemBoardInft = std::make_unique<InventoryItemBoardIntf>(
            utils::DBusHandler::getBus(), inventoryPath.c_str());
        return true;
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Failed to create Inventory Board interface for device {PATH}",
            "PATH", inventoryPath);
    }

    return false;
}

bool Terminus::parsePDRs()
{
    bool rc = true;
    std::vector<std::shared_ptr<pldm_numeric_sensor_value_pdr>>
        numericSensorPdrs{};
    std::vector<std::shared_ptr<pldm_compact_numeric_sensor_pdr>>
        compactNumericSensorPdrs{};
    std::vector<std::shared_ptr<pldm_numeric_effecter_value_pdr>>
        numericEffecterPdrs{};

    for (auto& pdr : pdrs)
    {
        auto pdrHdr = reinterpret_cast<pldm_pdr_hdr*>(pdr.data());
        if (pdrHdr->type == PLDM_SENSOR_AUXILIARY_NAMES_PDR ||
            pdrHdr->type == PLDM_EFFECTER_AUXILIARY_NAMES_PDR)
        {
            auto sensorAuxiliaryNames = parseSensorAuxiliaryNamesPDR(pdr);
            sensorAuxiliaryNamesTbl.emplace_back(
                std::move(sensorAuxiliaryNames));
        }
        else if (pdrHdr->type == PLDM_ENTITY_AUXILIARY_NAMES_PDR)
        {
            auto entityNames = parseEntityAuxiliaryNamesPDR(pdr);
            entityAuxiliaryNamesTbl.emplace_back(std::move(entityNames));
        }
        else if (pdrHdr->type == PLDM_NUMERIC_SENSOR_PDR)
        {
            auto parsedPdr = parseNumericSensorPDR(pdr);
            if (parsedPdr != nullptr)
            {
                numericSensorPdrs.emplace_back(std::move(parsedPdr));
            }
        }
        else if (pdrHdr->type == PLDM_COMPACT_NUMERIC_SENSOR_PDR)
        {
            auto parsedPdr = parseCompactNumericSensorPDR(pdr);
            if (parsedPdr != nullptr)
            {
                compactNumericSensorPdrs.emplace_back(std::move(parsedPdr));
                auto sensorAuxiliaryNames = parseCompactNumericSensorNames(pdr);
                if (sensorAuxiliaryNames != nullptr)
                {
                    sensorAuxiliaryNamesTbl.emplace_back(
                        std::move(sensorAuxiliaryNames));
                }
            }
        }
        else if (pdrHdr->type == PLDM_NUMERIC_EFFECTER_PDR)
        {
            auto parsedPdr = parseNumericEffecterPDR(pdr);
            if (parsedPdr != nullptr)
            {
                numericEffecterPdrs.emplace_back(std::move(parsedPdr));
            }
        }
        else
        {
            lg2::error("parsePDRs() Unsupported PDR with type {TYPE}", "TYPE",
                       pdrHdr->type);
            rc = false;
        }
    }

    auto tName = findTerminusName();
    if (tName != "")
    {
        terminusName = tName;
    }

    if (terminusName == "" &&
        (numericSensorPdrs.size() || compactNumericSensorPdrs.size()))
    {
        lg2::error(
            "Terminus ID {TID}: DOES NOT have name. Skip Adding sensors.",
            "TID", tid);
        return false;
    }

    if (createInventoryPath(terminusName))
    {
        lg2::error("Terminus ID {TID}: Created Inventory path.", "TID", tid);
    }

    for (auto pdr : numericSensorPdrs)
    {
        addNumericSensor(pdr);
    }

    for (auto pdr : compactNumericSensorPdrs)
    {
        addCompactNumericSensor(pdr);
    }

    for (auto pdr : numericEffecterPdrs)
    {
        addNumericEffecter(pdr);
    }

    return rc;
}

std::shared_ptr<SensorAuxiliaryNames>
    Terminus::getSensorAuxiliaryNames(SensorId id)
{
    for (auto sensorAuxiliaryNames : sensorAuxiliaryNamesTbl)
    {
        const auto& [sensorId, sensorCnt, sensorNames] = *sensorAuxiliaryNames;
        if (sensorId == id)
        {
            return sensorAuxiliaryNames;
        }
    }
    return nullptr;
}

std::shared_ptr<SensorAuxiliaryNames>
    Terminus::parseSensorAuxiliaryNamesPDR(const std::vector<uint8_t>& pdrData)
{
    constexpr uint8_t nullTerminator = 0;
    auto pdr = reinterpret_cast<const struct pldm_sensor_auxiliary_names_pdr*>(
        pdrData.data());
    const uint8_t* ptr = pdr->names;
    std::vector<std::vector<std::pair<NameLanguageTag, SensorName>>>
        sensorAuxNames{};
    char16_t alignedBuffer[PLDM_STR_UTF_16_MAX_LEN];
    for (int i = 0; i < pdr->sensor_count; i++)
    {
        const uint8_t nameStringCount = static_cast<uint8_t>(*ptr);
        ptr += sizeof(uint8_t);
        std::vector<std::pair<NameLanguageTag, SensorName>> nameStrings{};
        for (int j = 0; j < nameStringCount; j++)
        {
            std::string nameLanguageTag(reinterpret_cast<const char*>(ptr), 0,
                                        PLDM_STR_UTF_8_MAX_LEN);
            ptr += nameLanguageTag.size() + sizeof(nullTerminator);

            int u16NameStringLen = 0;
            for (int i = 0; ptr[i] != 0 || ptr[i + 1] != 0; i += 2)
            {
                u16NameStringLen++;
            }
            memset(alignedBuffer, 0,
                   PLDM_STR_UTF_16_MAX_LEN * sizeof(uint16_t));
            memcpy(alignedBuffer, ptr, u16NameStringLen * sizeof(uint16_t));
            std::u16string u16NameString(alignedBuffer, 0,
                                         PLDM_STR_UTF_16_MAX_LEN);
            ptr += (u16NameString.size() + sizeof(nullTerminator)) *
                   sizeof(uint16_t);
            std::transform(u16NameString.cbegin(), u16NameString.cend(),
                           u16NameString.begin(),
                           [](uint16_t utf16) { return be16toh(utf16); });
            std::string nameString =
                std::wstring_convert<std::codecvt_utf8_utf16<char16_t>,
                                     char16_t>{}
                    .to_bytes(u16NameString);
            nameStrings.emplace_back(
                std::make_pair(nameLanguageTag, nameString));
        }
        sensorAuxNames.emplace_back(nameStrings);
    }
    return std::make_shared<SensorAuxiliaryNames>(
        pdr->sensor_id, pdr->sensor_count, sensorAuxNames);
}

std::shared_ptr<EntityAuxiliaryNames>
    Terminus::parseEntityAuxiliaryNamesPDR(const std::vector<uint8_t>& pdrData)
{
    constexpr uint8_t nullTerminator = 0;
    auto pdr = reinterpret_cast<const struct pldm_entity_auxiliary_names_pdr*>(
        pdrData.data());
    const uint8_t* ptr = pdr->names;
    char16_t alignedBuffer[PLDM_STR_UTF_16_MAX_LEN];
    const uint8_t nameStringCount = static_cast<uint8_t>(*ptr);
    ptr += sizeof(uint8_t);
    std::vector<std::pair<NameLanguageTag, EntityName>> nameStrings{};
    for (int j = 0; j < nameStringCount; j++)
    {
        std::string nameLanguageTag(reinterpret_cast<const char*>(ptr), 0,
                                    PLDM_STR_UTF_8_MAX_LEN);
        ptr += nameLanguageTag.size() + sizeof(nullTerminator);

        int u16NameStringLen = 0;
        for (int i = 0; ptr[i] != 0 || ptr[i + 1] != 0; i += 2)
        {
            u16NameStringLen++;
        }
        memset(alignedBuffer, 0, PLDM_STR_UTF_16_MAX_LEN * sizeof(uint16_t));
        memcpy(alignedBuffer, ptr, u16NameStringLen * sizeof(uint16_t));
        std::u16string u16NameString(alignedBuffer, 0, PLDM_STR_UTF_16_MAX_LEN);
        ptr += (u16NameString.size() + sizeof(nullTerminator)) *
               sizeof(uint16_t);
        std::transform(u16NameString.cbegin(), u16NameString.cend(),
                       u16NameString.begin(),
                       [](uint16_t utf16) { return be16toh(utf16); });
        std::string nameString =
            std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t>{}
                .to_bytes(u16NameString);
        nameStrings.emplace_back(std::make_pair(nameLanguageTag, nameString));
    }

    EntityKey key{pdr->container.entity_type,
                  pdr->container.entity_instance_num,
                  pdr->container.entity_container_id};

    return std::make_shared<EntityAuxiliaryNames>(key, nameStrings);
}

std::shared_ptr<pldm_numeric_sensor_value_pdr>
    Terminus::parseNumericSensorPDR(const std::vector<uint8_t>& pdr)
{
    const uint8_t* ptr = pdr.data();
    auto parsedPdr = std::make_shared<pldm_numeric_sensor_value_pdr>();
    auto rc = decode_numeric_sensor_pdr_data(ptr, pdr.size(), parsedPdr.get());
    if (rc)
    {
        lg2::error("Failed to decode Numeric Sensor PDR data, error {RC} ",
                   "RC", rc);
        return nullptr;
    }
    return parsedPdr;
}

void Terminus::addNumericSensor(
    const std::shared_ptr<pldm_numeric_sensor_value_pdr> pdr)
{
    uint16_t sensorId = pdr->sensor_id;
    if (terminusName == "")
    {
        lg2::error(
            "Terminus ID {TID}: DOES NOT have name. Skip Adding sensors.",
            "TID", tid);
        return;
    }
    std::string sensorName = terminusName + "_" + "Sensor_" +
                             std::to_string(pdr->sensor_id);

    if (pdr->sensor_auxiliary_names_pdr)
    {
        auto sensorAuxiliaryNames = getSensorAuxiliaryNames(sensorId);
        if (sensorAuxiliaryNames)
        {
            const auto& [sensorId, sensorCnt,
                         sensorNames] = *sensorAuxiliaryNames;
            if (sensorCnt == 1)
            {
                for (const auto& [languageTag, name] : sensorNames[0])
                {
                    if (languageTag == "en" && name != "")
                    {
                        sensorName = terminusName + "_" + name;
                    }
                }
            }
        }
    }

    try
    {
        auto sensor = std::make_shared<NumericSensor>(
            tid, true, pdr, sensorName, inventoryPath);
        lg2::info("Created NumericSensor {NAME}", "NAME", sensorName);
        numericSensors.emplace_back(sensor);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Failed to create NumericSensor. error - {ERROR} sensorname - {NAME}",
            "ERROR", e, "NAME", sensorName);
    }
}

std::shared_ptr<SensorAuxiliaryNames>
    Terminus::parseCompactNumericSensorNames(const std::vector<uint8_t>& sPdr)
{
    std::string nameString;
    std::vector<std::vector<std::pair<NameLanguageTag, SensorName>>>
        sensorAuxNames{};
    std::vector<std::pair<NameLanguageTag, SensorName>> nameStrings{};
    auto pdr =
        reinterpret_cast<const pldm_compact_numeric_sensor_pdr*>(sPdr.data());

    if (sPdr.size() <
        (sizeof(pldm_compact_numeric_sensor_pdr) - sizeof(uint8_t)))
    {
        return nullptr;
    }

    if (pdr->sensor_name_length == 0)
    {
        return nullptr;
    }

    if (sPdr.size() < (sizeof(pldm_compact_numeric_sensor_pdr) -
                       sizeof(uint8_t) + pdr->sensor_name_length))
    {
        return nullptr;
    }

    std::string sTemp(reinterpret_cast<const char*>(pdr->sensor_name),
                      pdr->sensor_name_length);
    size_t pos = 0;
    while ((pos = sTemp.find(" ")) != std::string::npos)
    {
        sTemp.replace(pos, 1, "_");
    }
    nameString = sTemp;

    nameString.erase(nameString.find('\0'));
    nameStrings.emplace_back(std::make_pair("en", nameString));
    sensorAuxNames.emplace_back(nameStrings);

    return std::make_shared<SensorAuxiliaryNames>(pdr->sensor_id, 1,
                                                  sensorAuxNames);
}

std::shared_ptr<pldm_compact_numeric_sensor_pdr>
    Terminus::parseCompactNumericSensorPDR(const std::vector<uint8_t>& sPdr)
{
    std::string nameString;
    std::vector<std::pair<NameLanguageTag, SensorName>> nameStrings{};
    auto pdr =
        reinterpret_cast<const pldm_compact_numeric_sensor_pdr*>(sPdr.data());
    auto parsedPdr = std::make_shared<pldm_compact_numeric_sensor_pdr>();

    parsedPdr->hdr = pdr->hdr;
    parsedPdr->terminus_handle = pdr->terminus_handle;
    parsedPdr->sensor_id = pdr->sensor_id;
    parsedPdr->entity_type = pdr->entity_type;
    parsedPdr->entity_instance = pdr->entity_instance;
    parsedPdr->container_id = pdr->container_id;
    parsedPdr->sensor_name_length = pdr->sensor_name_length;
    parsedPdr->base_unit = pdr->base_unit;
    parsedPdr->unit_modifier = pdr->unit_modifier;
    parsedPdr->occurrence_rate = pdr->occurrence_rate;
    parsedPdr->range_field_support = pdr->range_field_support;
    parsedPdr->warning_high = pdr->warning_high;
    parsedPdr->warning_low = pdr->warning_low;
    parsedPdr->critical_high = pdr->critical_high;
    parsedPdr->critical_low = pdr->critical_low;
    parsedPdr->fatal_high = pdr->fatal_high;
    parsedPdr->fatal_low = pdr->fatal_low;
    return parsedPdr;
}

void Terminus::addCompactNumericSensor(
    const std::shared_ptr<pldm_compact_numeric_sensor_pdr> pdr)
{
    uint16_t sensorId = pdr->sensor_id;
    if (terminusName == "")
    {
        lg2::error(
            "Terminus ID {TID}: DOES NOT have name. Skip Adding sensors.",
            "TID", tid);
        return;
    }
    std::string sensorName = terminusName + "_" + "Sensor_" +
                             std::to_string(pdr->sensor_id);

    auto sensorAuxiliaryNames = getSensorAuxiliaryNames(sensorId);
    if (sensorAuxiliaryNames)
    {
        const auto& [sensorId, sensorCnt, sensorNames] = *sensorAuxiliaryNames;
        if (sensorCnt == 1)
        {
            for (const auto& [languageTag, name] : sensorNames[0])
            {
                if (languageTag == "en" && name != "")
                {
                    sensorName = terminusName + "_" + name;
                }
            }
        }
    }

    try
    {
        auto sensor = std::make_shared<NumericSensor>(
            tid, true, pdr, sensorName, inventoryPath);
        lg2::info("Created Compact NumericSensor {NAME}", "NAME", sensorName);
        numericSensors.emplace_back(sensor);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Failed to create Compact NumericSensor. error - {ERROR} sensorname - {NAME}",
            "ERROR", e, "NAME", sensorName);
    }
}

std::shared_ptr<pldm_numeric_effecter_value_pdr>
    Terminus::parseNumericEffecterPDR(const std::vector<uint8_t>& pdr)
{
    const uint8_t* ptr = pdr.data();
    auto parsedPdr = std::make_shared<pldm_numeric_effecter_value_pdr>();
    auto rc = decode_numeric_effecter_pdr_data(ptr, pdr.size(),
                                               parsedPdr.get());
    if (rc)
    {
        lg2::error("Failed to decode Numeric effecter PDR date, error {RC} ",
                   "RC", rc);
        return nullptr;
    }
    return parsedPdr;
}

void Terminus::addNumericEffecter(
    const std::shared_ptr<pldm_numeric_effecter_value_pdr> pdr)
{
    uint16_t sensorId = pdr->effecter_id;
    if (terminusName == "")
    {
        lg2::error(
            "Terminus ID {TID}: DOES NOT have name. Skip Adding effecters.",
            "TID", tid);
        return;
    }
    std::string effecterName = terminusName + "_" + "Effecter_" +
                               std::to_string(sensorId);

    auto sensorAuxiliaryNames = getSensorAuxiliaryNames(sensorId);
    if (sensorAuxiliaryNames)
    {
        const auto& [sensorId, sensorCnt,
                     effecterNames] = *sensorAuxiliaryNames;
        if (sensorCnt == 1)
        {
            for (const auto& [languageTag, name] : effecterNames[0])
            {
                if (languageTag == "en" && name != "")
                {
                    effecterName = terminusName + "_" + name;
                }
            }
        }
    }

    try
    {
        auto sensor = std::make_shared<NumericSensor>(
            tid, true, pdr, effecterName, inventoryPath);
        lg2::info("Created NumericEffecter {NAME}", "NAME", effecterName);
        numericSensors.emplace_back(sensor);
    }
    catch (const std::exception& e)
    {
        lg2::error(
            "Failed to create NumericEffecter. error - {ERROR} effecterName - {NAME}",
            "ERROR", e, "NAME", effecterName);
    }
}

} // namespace platform_mc

} // namespace pldm
