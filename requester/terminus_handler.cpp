#include "config.h"

#include "terminus_handler.hpp"

namespace pldm
{

namespace terminus
{

TerminusHandler::TerminusHandler(
    uint8_t eid, sdeventplus::Event& event, sdbusplus::bus::bus& bus,
    pldm_pdr* repo, pldm_entity_association_tree* entityTree,
    pldm_entity_association_tree* bmcEntityTree,
    pldm::requester::Handler<pldm::requester::Request>* handler,
    pldm::InstanceIdDb& instanceIdDb) :
    eid(eid),
    bus(bus), event(event), repo(repo), entityTree(entityTree),
    bmcEntityTree(bmcEntityTree), handler(handler),
    instanceIdDb(instanceIdDb)
{}

TerminusHandler::~TerminusHandler()
{}

void TerminusHandler::discoveryTerminus()
{
    return;
}

} // namespace terminus

} // namespace pldm
