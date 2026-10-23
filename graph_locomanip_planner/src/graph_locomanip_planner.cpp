#include <graph_locomanip_planner/contact_graph.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <algorithm>
#include <future>
#include <iostream>
#include <stdexcept>

namespace graph_locomanip_planner {
  ContactPlanner::GLMPParam::GLMPParam() {
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
      models.push_back(this->object[i]);
    }
    return models;
  }
  bool ContactPlanner::solve() {
    this->param.getModels();
    if(!this->param.currentContactState || !this->param.goalContactState)
      throw std::invalid_argument("ContactPlanner requires start and goal");
    if(this->threads() == 0) throw std::invalid_argument("ContactPlanner requires at least one worker");
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
        node->state().contacts.erase(node->state().contacts.begin()+i); result.push_back(node);
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
        if((dynamic.bodyName == contact.c1.bodyName && dynamic.linkName == contact.c1.linkName) ||
           (dynamic.bodyName == contact.c2.bodyName && dynamic.linkName == contact.c2.linkName)) used = true;
      }
      if(used) continue;
      const std::size_t index = getModelIndex(dynamic,models);
      const Eigen::Vector3d rootPosition = data[index]->oMi[1].translation();
      for(std::size_t j = 0; j < check->param.contactStaticCandidates.size(); ++j) {
        const ContactCandidate& fixed = *check->param.contactStaticCandidates[j];
        if(!fixed.allowedDynamicLinks.empty() && std::find(fixed.allowedDynamicLinks.begin(),
          fixed.allowedDynamicLinks.end(),dynamic.linkName) == fixed.allowedDynamicLinks.end()) continue;
        if((rootPosition-fixed.localPose.translation()).norm() > check->param.addCandidateDistance) continue;
        std::shared_ptr<ContactNode> node = std::make_shared<ContactNode>(); node->state() = state;
        node->state().contacts.push_back(Contact(dynamic,fixed)); result.push_back(node);
      }
      for(std::size_t j = i+1; j < check->param.contactDynamicCandidates.size(); ++j) {
        const ContactCandidate& other = *check->param.contactDynamicCandidates[j];
        if(dynamic.bodyName == other.bodyName && dynamic.linkName == other.linkName) continue;
        bool otherUsed = false;
        for(std::size_t k = 0; k < state.contacts.size(); ++k) {
          if((other.bodyName == state.contacts[k].c1.bodyName && other.linkName == state.contacts[k].c1.linkName) ||
             (other.bodyName == state.contacts[k].c2.bodyName && other.linkName == state.contacts[k].c2.linkName)) otherUsed = true;
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
