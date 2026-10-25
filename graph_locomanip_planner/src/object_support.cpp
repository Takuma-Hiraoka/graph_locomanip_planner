#include <graph_locomanip_planner/contact_graph.h>
#include <algorithm>
#include <stdexcept>

namespace graph_locomanip_planner {
  namespace {
    bool robotContact(const Contact& contact, const ContactCandidate& site,
      const std::string& robotName, ContactCandidate& robotSite) {
      if(contact.c1 == site && !contact.c2.isStatic && contact.c2.bodyName == robotName) {
        robotSite = contact.c2; return true;
      }
      if(contact.c2 == site && !contact.c1.isStatic && contact.c1.bodyName == robotName) {
        robotSite = contact.c1; return true;
      }
      return false;
    }
  }
  bool isObjectGrasped(const ContactState& state, const ObjectContactRequirement& requirement,
    const std::string& robotName) {
    for(std::size_t pair = 0; pair < requirement.liftContactPairs.size(); ++pair) {
      for(std::size_t i = 0; i < state.contacts.size(); ++i) {
        ContactCandidate first;
        if(!robotContact(state.contacts[i],requirement.liftContactPairs[pair].first,robotName,first)) continue;
        for(std::size_t j = 0; j < state.contacts.size(); ++j) {
          ContactCandidate second;
          if(i != j && robotContact(state.contacts[j],requirement.liftContactPairs[pair].second,robotName,second) &&
             !(first == second)) return true;
        }
      }
    }
    return false;
  }
  bool isObjectStaticallySupported(const ContactState& state, const ObjectContactRequirement& requirement) {
    for(std::size_t i = 0; i < state.contacts.size(); ++i) {
      const Contact& contact = state.contacts[i];
      for(std::size_t j = 0; j < requirement.supportCandidates.size(); ++j) {
        const ContactCandidate& site = requirement.supportCandidates[j];
        if((contact.c1 == site && contact.c2.isStatic) || (contact.c2 == site && contact.c1.isStatic)) return true;
      }
    }
    return false;
  }
  bool ContactPlanner::checkObjectFeasibility(const ContactState& state, const GLMPParam& parameters) const {
    if(parameters.objectContactRequirements.size() != parameters.object.size())
      throw std::invalid_argument("Each object requires an ObjectContactRequirement");
    for(std::size_t i = 0; i < parameters.object.size(); ++i) {
      const ObjectContactRequirement* requirement = nullptr;
      for(std::size_t j = 0; j < parameters.objectContactRequirements.size(); ++j) {
        if(parameters.objectContactRequirements[j].model == parameters.object[i]) {
          if(requirement) throw std::invalid_argument("Duplicate object contact requirement");
          requirement = &parameters.objectContactRequirements[j];
        }
      }
      if(!requirement) throw std::invalid_argument("Missing object contact requirement");
      if(!isObjectStaticallySupported(state,*requirement) && !isObjectGrasped(state,*requirement,parameters.robot->name)) return false;
    }
    return true;
  }
  std::shared_ptr<ik_constraint_sp::SPConstraint> ContactPlanner::createSPConstraint(
    const ContactState& supportState, const GLMPParam& parameters) const {
    if(!this->checkObjectFeasibility(supportState,parameters)) return nullptr;
    std::shared_ptr<ik_constraint_sp::SPConstraint> result = parameters.spConstraint ?
      std::make_shared<ik_constraint_sp::SPConstraint>(*parameters.spConstraint) : std::make_shared<ik_constraint_sp::SPConstraint>();
    std::vector<std::shared_ptr<const pinocchio::Model> > COMModels{parameters.robot};
    for(std::size_t i = 0; i < parameters.objectContactRequirements.size(); ++i) {
      const ObjectContactRequirement& requirement = parameters.objectContactRequirements[i];
      if(!isObjectStaticallySupported(supportState,requirement) &&
         isObjectGrasped(supportState,requirement,parameters.robot->name)) COMModels.push_back(requirement.model);
    }
    result->setModels(COMModels);
    std::vector<ik_constraint_sp::SupportPolygon> polygons;
    for(std::size_t i = 0; i < supportState.contacts.size(); ++i) {
      const Contact& contact = supportState.contacts[i];
      const ContactCandidate* robotSite = nullptr;
      const ContactCandidate* supportSite = nullptr;
      if(!contact.c1.isStatic && contact.c1.bodyName == parameters.robot->name) {
        robotSite = &contact.c1; supportSite = &contact.c2;
      } else if(!contact.c2.isStatic && contact.c2.bodyName == parameters.robot->name) {
        robotSite = &contact.c2; supportSite = &contact.c1;
      }
      if(!robotSite || robotSite->supportPolygon.empty()) continue;
      bool grounded = supportSite->isStatic;
      if(!grounded) {
        for(std::size_t j = 0; j < parameters.objectContactRequirements.size(); ++j) {
          const ObjectContactRequirement& requirement = parameters.objectContactRequirements[j];
          if(requirement.model->name == supportSite->bodyName && isObjectStaticallySupported(supportState,requirement)) grounded = true;
        }
      }
      // A grasped, airborne object is not an external support for its carrier.
      if(!grounded) continue;
      ik_constraint_sp::SupportPolygon polygon;
      polygon.model = parameters.robot;
      polygon.jointId = getJointId(*robotSite,*parameters.robot);
      polygon.localPose = getJointLocalPose(*robotSite,*parameters.robot);
      polygon.vertices = robotSite->supportPolygon;
      polygons.push_back(polygon);
    }
    if(polygons.empty()) return nullptr;
    result->setSupportPolygons(polygons);
    return result;
  }
}
