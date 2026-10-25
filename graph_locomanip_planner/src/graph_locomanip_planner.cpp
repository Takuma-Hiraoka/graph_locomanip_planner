#include <graph_locomanip_planner/contact_graph.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <algorithm>
#include <future>
#include <iostream>
#include <stdexcept>

namespace graph_locomanip_planner {
  ContactPlanner::GLMPParam::GLMPParam() {
    this->spConstraint = std::make_shared<ik_constraint_sp::SPConstraint>();
    this->spConstraint->setTolerance(0.005);
    this->spConstraint->setPrecision(1e-3);
    this->pikParam.maxIteration = 100;
    this->pikParam.checkFinalState = true;
    this->pikParam.we = 1e2; this->pikParam.wmax = 1e1;
    this->pikParam.convergeThre = 1e-5;
    this->gikParam.ikParam = this->pikParam;
    this->gikParam.timeout = 1.0; this->gikParam.threads = 1;
    this->gikParam.maxTranslation = 1.5; this->gikParam.delta = 0.05;
    this->gikParam.goalBias = 0.2;
  }
  std::vector<std::shared_ptr<pinocchio::Model> > ContactPlanner::GLMPParam::getModels() const {
    if(!this->robot) throw std::invalid_argument("ContactPlanner requires a robot model");
    std::vector<std::shared_ptr<pinocchio::Model> > models{this->robot};
    for(std::size_t i = 0; i < this->object.size(); ++i) {
      if(!this->object[i]) throw std::invalid_argument("ContactPlanner object model is null");
      for(std::size_t j = 0; j < models.size(); ++j) {
        if(models[j] == this->object[i] || models[j]->name == this->object[i]->name)
          throw std::invalid_argument("ContactPlanner model pointers and names must be unique");
      }
      models.push_back(this->object[i]);
    }
    return models;
  }
  bool ContactPlanner::solve() {
    this->param.getModels();
    if(!this->param.currentContactState || !this->param.goalContactState)
      throw std::invalid_argument("ContactPlanner requires start and goal");
    if(this->threads() == 0) throw std::invalid_argument("ContactPlanner requires at least one worker");
    for(std::size_t i = 0; i < this->param.objectContactRequirements.size(); ++i) {
      const ObjectContactRequirement& requirement = this->param.objectContactRequirements[i];
      if(!requirement.model) throw std::invalid_argument("Null object contact requirement model");
      std::vector<ContactCandidate> sites = requirement.supportCandidates;
      for(std::size_t j = 0; j < requirement.liftContactPairs.size(); ++j) {
        if(requirement.liftContactPairs[j].first == requirement.liftContactPairs[j].second)
          throw std::invalid_argument("A lift pair must contain two different object sites");
        sites.push_back(requirement.liftContactPairs[j].first);
        sites.push_back(requirement.liftContactPairs[j].second);
      }
      for(std::size_t j = 0; j < sites.size(); ++j) {
        if(sites[j].isStatic || sites[j].bodyName != requirement.model->name)
          throw std::invalid_argument("Object contact requirement site belongs to the wrong model");
        getJointId(sites[j],*requirement.model);
      }
    }
    if(!this->checkObjectFeasibility(*this->param.currentContactState,this->param)) return false;
    this->graph().clear(); this->setGoal(nullptr);
    std::shared_ptr<ContactNode> start = std::make_shared<ContactNode>();
    start->state() = *this->param.currentContactState;
    this->graph().push_back(start);
    std::vector<std::shared_ptr<graph_search::Planner::TransitionCheckParam> > workers;
    for(unsigned int i = 0; i < this->threads(); ++i) workers.push_back(this->generateCheckParam());
    this->calcHeuristic(workers[0],start);
    unsigned long extended = 0;
    // Bounded batches avoid spinning forever when no unexpanded node remains.
    while(extended < this->maxExtendNum()) {
      std::vector<std::shared_ptr<graph_search::Node> > nodes;
      for(std::size_t i = 0; i < this->graph().size() && nodes.size() < workers.size() && extended < this->maxExtendNum(); ++i) {
        if(this->graph()[i]->extended()) continue;
        this->graph()[i]->extended() = true;
        nodes.push_back(this->graph()[i]); ++extended;
        this->preCheckTransition(workers[nodes.size()-1],nodes.back());
      }
      if(nodes.empty()) return false;
      std::vector<std::future<bool> > results;
      for(std::size_t i = 0; i < nodes.size(); ++i)
        results.push_back(std::async(std::launch::async,[this,&workers,i]() { return this->checkTransition(workers[i]); }));
      // Always join all workers before returning, including on exceptions.
      std::vector<bool> solved(nodes.size(),false);
      for(std::size_t i = 0; i < nodes.size(); ++i) solved[i] = results[i].get();
      for(std::size_t i = 0; i < nodes.size(); ++i) {
        if(this->debugLevel() >= 1) std::cerr << "ContactPlanner: extended " << extended << ", graph " << this->graph().size() << ", feasible " << solved[i] << std::endl;
        if(!solved[i]) continue;
        this->postCheckTransition(workers[i],nodes[i]);
        if(this->isGoalSatisfied(workers[i])) { this->setGoal(nodes[i]); return true; }
        std::vector<std::shared_ptr<graph_search::Node> > adjacent = this->gatherAdjacentNodes(workers[i]);
        for(std::size_t j = 0; j < adjacent.size(); ++j) {
          adjacent[j]->parent() = nodes[i]; this->calcHeuristic(workers[i],adjacent[j]);
        }
        std::stable_sort(adjacent.begin(),adjacent.end(),[](const std::shared_ptr<graph_search::Node>& a,
          const std::shared_ptr<graph_search::Node>& b) { return a->heuristic() < b->heuristic(); });
        this->addNodes2Graph(adjacent);
      }
    }
    return false;
  }
  std::shared_ptr<graph_search::Planner::TransitionCheckParam> ContactPlanner::generateCheckParam() {
    std::shared_ptr<ContactTransitionCheckParam> result = std::make_shared<ContactTransitionCheckParam>();
    result->param = this->param;
    if(this->param.spConstraint) result->param.spConstraint = std::make_shared<ik_constraint_sp::SPConstraint>(*this->param.spConstraint);
    result->param.constraints.clear(); result->param.rejections.clear(); result->param.nominals.clear();
    for(std::size_t i = 0; i < this->param.constraints.size(); ++i) result->param.constraints.push_back(this->param.constraints[i]->clone());
    for(std::size_t i = 0; i < this->param.rejections.size(); ++i) result->param.rejections.push_back(this->param.rejections[i]->clone());
    for(std::size_t i = 0; i < this->param.nominals.size(); ++i) result->param.nominals.push_back(this->param.nominals[i]->clone());
    result->param.pikParam.viewer = nullptr; result->param.gikParam.viewer = nullptr;
    result->param.gikParam.ikParam.viewer = nullptr;
    return result;
  }
  void ContactPlanner::preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
    std::shared_ptr<graph_search::Node> node) {
    std::shared_ptr<ContactTransitionCheckParam> check = std::static_pointer_cast<ContactTransitionCheckParam>(checkParam);
    check->postState = std::static_pointer_cast<ContactNode>(node)->state();
    check->preState = node->parent().expired() ? check->postState : std::static_pointer_cast<ContactNode>(node->parent().lock())->state();
  }
  void ContactPlanner::postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
    std::shared_ptr<graph_search::Node> node) {
    std::static_pointer_cast<ContactNode>(node)->state() = std::static_pointer_cast<ContactTransitionCheckParam>(checkParam)->postState;
  }
  bool ContactPlanner::checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> check = std::static_pointer_cast<ContactTransitionCheckParam>(checkParam);
    return this->checkTransitionImpl(check->preState,check->postState,check->param);
  }
  bool ContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    const ContactState& state = std::static_pointer_cast<ContactTransitionCheckParam>(checkParam)->postState;
    for(std::size_t i = 0; i < this->param.goalContactState->contacts.size(); ++i)
      if(std::find(state.contacts.begin(),state.contacts.end(),this->param.goalContactState->contacts[i]) == state.contacts.end()) return false;
    return true;
  }
  void ContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam>, std::shared_ptr<graph_search::Node> node) {
    const ContactState& state = std::static_pointer_cast<ContactNode>(node)->state();
    double missing = 0.0;
    for(std::size_t i = 0; i < this->param.goalContactState->contacts.size(); ++i)
      if(std::find(state.contacts.begin(),state.contacts.end(),this->param.goalContactState->contacts[i]) == state.contacts.end()) missing += 1.0;
    // Grasping is a prerequisite for moving a supported object to another site.
    for(std::size_t i = 0; i < this->param.objectContactRequirements.size(); ++i) {
      const ObjectContactRequirement& requirement = this->param.objectContactRequirements[i];
      bool objectGoalMissing = false;
      for(std::size_t j = 0; j < this->param.goalContactState->contacts.size(); ++j) {
        const Contact& goal = this->param.goalContactState->contacts[j];
        if((!goal.c1.isStatic && goal.c1.bodyName == requirement.model->name) ||
           (!goal.c2.isStatic && goal.c2.bodyName == requirement.model->name)) {
          if(std::find(state.contacts.begin(),state.contacts.end(),goal) == state.contacts.end()) objectGoalMissing = true;
        }
      }
      if(!objectGoalMissing) continue;
      double deficit = 2.0;
      for(std::size_t j = 0; j < requirement.liftContactPairs.size(); ++j) {
        const ContactCandidate sites[2] = {requirement.liftContactPairs[j].first,requirement.liftContactPairs[j].second};
        double count = 0.0;
        for(int k = 0; k < 2; ++k) {
          bool found = false;
          for(std::size_t c = 0; c < state.contacts.size(); ++c) {
            const Contact& contact = state.contacts[c];
            if((contact.c1 == sites[k] && !contact.c2.isStatic && contact.c2.bodyName == this->param.robot->name) ||
               (contact.c2 == sites[k] && !contact.c1.isStatic && contact.c1.bodyName == this->param.robot->name)) found = true;
          }
          if(!found) count += 1.0;
        }
        deficit = std::min(deficit,count);
      }
      missing += 0.25*deficit;
      if(isObjectStaticallySupported(state,requirement)) missing += 0.1;
    }
    node->heuristic() = missing;
  }
  std::vector<std::shared_ptr<graph_search::Node> > ContactPlanner::gatherAdjacentNodes(
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> check = std::static_pointer_cast<ContactTransitionCheckParam>(checkParam);
    const ContactState& state = check->postState;
    std::vector<std::shared_ptr<graph_search::Node> > result;
    if(state.contacts.size() >= 2) {
      for(std::size_t i = 0; i < state.contacts.size(); ++i) {
        std::shared_ptr<ContactNode> node = std::make_shared<ContactNode>(); node->state() = state;
        node->state().contacts.erase(node->state().contacts.begin()+i);
        if(this->checkObjectFeasibility(node->state(),check->param)) result.push_back(node);
      }
    }
    const std::vector<std::shared_ptr<pinocchio::Model> > models = check->param.getModels();
    std::vector<std::shared_ptr<pinocchio::Data> > data = ik_constraint::IKConstraint::createDataVector(models);
    for(std::size_t i = 0; i < data.size(); ++i) pinocchio::forwardKinematics(*models[i],*data[i],state.frame[i]);
    for(std::size_t i = 0; i < check->param.contactDynamicCandidates.size(); ++i) {
      const ContactCandidate& dynamic = *check->param.contactDynamicCandidates[i];
      bool used = false;
      for(std::size_t j = 0; j < state.contacts.size(); ++j) {
        const Contact& contact = state.contacts[j];
        if(dynamic == contact.c1 || dynamic == contact.c2) used = true;
      }
      if(used) continue;
      const std::size_t index = getModelIndex(dynamic,models);
      const Eigen::Vector3d rootPosition = data[index]->oMi[1].translation();
      for(std::size_t j = 0; j < check->param.contactStaticCandidates.size(); ++j) {
        const ContactCandidate& fixed = *check->param.contactStaticCandidates[j];
        if(!dynamic.allowStaticContact) continue;
        if(!dynamic.allowedStaticLinks.empty() && std::find(dynamic.allowedStaticLinks.begin(),
          dynamic.allowedStaticLinks.end(),fixed.linkName) == dynamic.allowedStaticLinks.end()) continue;
        if(!fixed.allowedDynamicLinks.empty() && std::find(fixed.allowedDynamicLinks.begin(),
          fixed.allowedDynamicLinks.end(),dynamic.linkName) == fixed.allowedDynamicLinks.end()) continue;
        if((rootPosition-fixed.localPose.translation()).norm() > check->param.addCandidateDistance) continue;
        std::shared_ptr<ContactNode> node = std::make_shared<ContactNode>(); node->state() = state;
        node->state().contacts.push_back(Contact(dynamic,fixed)); result.push_back(node);
      }
      for(std::size_t j = i+1; j < check->param.contactDynamicCandidates.size(); ++j) {
        const ContactCandidate& other = *check->param.contactDynamicCandidates[j];
        if(!dynamic.allowDynamicContact || !other.allowDynamicContact || dynamic.bodyName == other.bodyName) continue;
        if(dynamic.bodyName != check->param.robot->name && other.bodyName != check->param.robot->name) continue;
        if(!dynamic.allowedDynamicLinks.empty() && std::find(dynamic.allowedDynamicLinks.begin(),
          dynamic.allowedDynamicLinks.end(),other.linkName) == dynamic.allowedDynamicLinks.end()) continue;
        if(!other.allowedDynamicLinks.empty() && std::find(other.allowedDynamicLinks.begin(),
          other.allowedDynamicLinks.end(),dynamic.linkName) == other.allowedDynamicLinks.end()) continue;
        bool otherUsed = false;
        for(std::size_t k = 0; k < state.contacts.size(); ++k) {
          if(other == state.contacts[k].c1 || other == state.contacts[k].c2) otherUsed = true;
        }
        if(otherUsed || (getWorldPose(dynamic,models,data).translation()-getWorldPose(other,models,data).translation()).norm() > check->param.addCandidateDistance) continue;
        std::shared_ptr<ContactNode> node = std::make_shared<ContactNode>(); node->state() = state;
        node->state().contacts.push_back(Contact(dynamic,other)); result.push_back(node);
      }
    }
    return result;
  }
  void ContactPlanner::addNodes2Graph(std::vector<std::shared_ptr<graph_search::Node> >& nodes) {
    std::vector<std::shared_ptr<graph_search::Node> > unique;
    for(std::size_t i = 0; i < nodes.size(); ++i) {
      bool found = false;
      for(std::size_t j = 0; j < this->graph().size() && !found; ++j)
        found = std::static_pointer_cast<ContactNode>(nodes[i])->state() == std::static_pointer_cast<ContactNode>(this->graph()[j])->state();
      for(std::size_t j = 0; j < unique.size() && !found; ++j)
        found = std::static_pointer_cast<ContactNode>(nodes[i])->state() == std::static_pointer_cast<ContactNode>(unique[j])->state();
      if(!found) unique.push_back(nodes[i]);
    }
    nodes = unique;
    graph_search::Planner::addNodes2Graph(nodes);
  }
  void ContactPlanner::goalPath(std::vector<ContactState>& path) {
    path.clear(); std::shared_ptr<graph_search::Node> node = this->goal();
    while(node) { path.push_back(std::static_pointer_cast<ContactNode>(node)->state()); node = node->parent().lock(); }
    std::reverse(path.begin(),path.end());
  }
}
