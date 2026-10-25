#include <graph_locomanip_planner/contact_graph.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/CollisionConstraint.h>
#include <ik_constraint_esdf/EsdfCollisionConstraint.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace graph_locomanip_planner {
  std::size_t getModelIndex(const ContactCandidate& candidate,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) {
    for(std::size_t i = 0; i < models.size(); ++i) if(models[i]->name == candidate.bodyName) return i;
    throw std::invalid_argument("ContactPlanner: model not found: " + candidate.bodyName);
  }
  pinocchio::JointIndex getJointId(const ContactCandidate& candidate, const pinocchio::Model& model) {
    if(candidate.isStatic) return 0;
    if(model.existFrame(candidate.linkName)) return model.frames[model.getFrameId(candidate.linkName)].parentJoint;
    if(model.existJointName(candidate.linkName)) return model.getJointId(candidate.linkName);
    throw std::invalid_argument("ContactPlanner: link or joint not found: " + candidate.linkName);
  }
  pinocchio::SE3 getJointLocalPose(const ContactCandidate& candidate, const pinocchio::Model& model) {
    if(!candidate.isStatic && model.existFrame(candidate.linkName))
      return model.frames[model.getFrameId(candidate.linkName)].placement * candidate.localPose;
    return candidate.localPose;
  }
  pinocchio::SE3 getWorldPose(const ContactCandidate& candidate,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data) {
    if(candidate.isStatic) return candidate.localPose;
    const std::size_t i = getModelIndex(candidate,models);
    return data[i]->oMi[getJointId(candidate,*models[i])] * getJointLocalPose(candidate,*models[i]);
  }
  namespace {
    void fixContactYaw(Contact& contact,
      const std::vector<std::shared_ptr<pinocchio::Model> >& models,
      const std::vector<std::shared_ptr<pinocchio::Data> >& data) {
      if(contact.yawFixed) return;
      const ContactCandidate& a = contact.c1.isStatic ? contact.c2 : contact.c1;
      const ContactCandidate& b = contact.c1.isStatic ? contact.c1 : contact.c2;
      const Eigen::Matrix3d relativeRotation = getWorldPose(b,models,data).rotation().transpose()*
        getWorldPose(a,models,data).rotation();
      contact.yaw = std::atan2(relativeRotation(1,0),relativeRotation(0,0));
      contact.yawFixed = true;
    }

    bool matches(const ContactCandidate& candidate, const std::shared_ptr<const pinocchio::Model>& model,
      pinocchio::JointIndex joint) {
      return candidate.isStatic ? joint == 0 : candidate.bodyName == model->name && getJointId(candidate,*model) == joint;
    }
    bool matchesPair(const Contact& contact, const std::shared_ptr<ik_constraint::CollisionConstraint>& collision) {
      return (matches(contact.c1,collision->getAModel(),collision->getAJointId()) && matches(contact.c2,collision->getBModel(),collision->getBJointId())) ||
        (matches(contact.c2,collision->getAModel(),collision->getAJointId()) && matches(contact.c1,collision->getBModel(),collision->getBJointId()));
    }
    bool skipCollision(const std::shared_ptr<ik_constraint::IKConstraint>& constraint,
      const ContactState& support, const Contact& move, const IKState& state) {
      const std::shared_ptr<ik_constraint::CollisionConstraint> collision =
        std::dynamic_pointer_cast<ik_constraint::CollisionConstraint>(constraint);
      if(!collision) return false;
      const bool distanceField = static_cast<bool>(std::dynamic_pointer_cast<ik_constraint_esdf::EsdfCollisionConstraint>(constraint));
      for(std::size_t i = 0; i < support.contacts.size(); ++i) {
        const Contact& contact = support.contacts[i];
        if(distanceField) {
          if((matches(contact.c1,collision->getAModel(),collision->getAJointId()) && contact.c2.isStatic) ||
             (matches(contact.c2,collision->getAModel(),collision->getAJointId()) && contact.c1.isStatic)) return true;
        } else if(matchesPair(contact,collision)) return true;
      }
      if(state != IKState::ATTACH_FIXED && state != IKState::DETACH_FIXED) return false;
      if(distanceField) return (move.c1.isStatic || move.c2.isStatic) &&
        (matches(move.c1,collision->getAModel(),collision->getAJointId()) || matches(move.c2,collision->getAModel(),collision->getAJointId()));
      return matchesPair(move,collision);
    }
    std::shared_ptr<ik_constraint::PositionConstraint> makePosition(const Contact& contact,
      const ContactPlanner::GLMPParam& param, double clearance) {
      const ContactCandidate& a = contact.c1.isStatic ? contact.c2 : contact.c1;
      const ContactCandidate& b = contact.c1.isStatic ? contact.c1 : contact.c2;
      if(a.isStatic) throw std::invalid_argument("ContactPlanner: static-static contact");
      const std::vector<std::shared_ptr<pinocchio::Model> > models = param.getModels();
      const std::size_t ai = getModelIndex(a,models);
      std::shared_ptr<ik_constraint::PositionConstraint> result;
      if(b.isStatic) result = std::make_shared<ik_constraint::PositionConstraint>(models[ai]);
      else result = std::make_shared<ik_constraint::PositionConstraint>(models[ai],models[getModelIndex(b,models)]);
      result->setAJointId(getJointId(a,*models[ai])); result->setALocalPose(getJointLocalPose(a,*models[ai]));
      const pinocchio::Model& bModel = b.isStatic ? *models[ai] : *models[getModelIndex(b,models)];
      result->setBJointId(getJointId(b,bModel));
      pinocchio::SE3 target = getJointLocalPose(b,bModel);
      const Eigen::Matrix3d evalRotation = target.rotation();
      target.translation() += clearance * target.rotation().col(2);
      if(contact.yawFixed) target.rotation() *= Eigen::AngleAxisd(contact.yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      result->setBLocalPose(target); result->setPrecision(param.contactPrecision);
      result->setEvalModel(result->getBModel());
      result->setEvalJointId(result->getBJointId());
      result->setEvalLocalR(evalRotation);
      Eigen::Matrix<double,6,1> weight = a.weight.cwiseMin(b.weight);
      weight[5] = contact.yawFixed ? 1.0 : 0.0;
      result->setWeight(weight);
      result->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
      return result;
    }

  }
  bool ContactPlanner::solveContactIK(const ContactState& supportState, Contact& moveContact, ContactState& postState,
    const IKState& state, std::vector<Eigen::VectorXd>& configurations, const GLMPParam& parameters) {
    const std::vector<std::shared_ptr<pinocchio::Model> > models = parameters.getModels();
    const std::vector<Eigen::VectorXd> initial = configurations;
    std::shared_ptr<ik_constraint_sp::SPConstraint> balance = this->createSPConstraint(supportState,parameters);
    if(!balance) return false;
    std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > levels(4);
    levels[0].push_back(balance);
    if(state == IKState::ATTACH_FIXED) {
      // On touchdown the supported object's mass leaves the robot COM.
      // Enforce balance on both sides of that load change, including when
      // this node already satisfies the goal and no next IK phase is run.
      std::shared_ptr<ik_constraint_sp::SPConstraint> finalBalance = this->createSPConstraint(postState,parameters);
      if(!finalBalance) return false;
      if(finalBalance->getCOMModels() != balance->getCOMModels()) levels[0].push_back(finalBalance);
    }
    for(std::size_t i = 0; i < parameters.constraints.size(); ++i)
      if(!skipCollision(parameters.constraints[i],supportState,moveContact,state)) levels[0].push_back(parameters.constraints[i]);
    for(std::size_t i = 0; i < supportState.contacts.size(); ++i) {
      if(!moveContact.c1.linkName.empty() && supportState.contacts[i] == moveContact) continue;
      levels[1].push_back(makePosition(supportState.contacts[i],parameters,0.0));
    }
    std::shared_ptr<ik_constraint::PositionConstraint> moving;
    if(!moveContact.c1.linkName.empty()) {
      const double clearance = state == IKState::DETACH_FIXED || state == IKState::ATTACH_PRE ? parameters.contactClearance : 0.0;
      moving = makePosition(moveContact,parameters,clearance); levels[2].push_back(moving);
    }
    levels[3] = parameters.nominals;
    std::shared_ptr<std::vector<std::vector<Eigen::VectorXd> > > path = std::make_shared<std::vector<std::vector<Eigen::VectorXd> > >();
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    bool solved = prioritized_inverse_kinematics_solver::solveIKLoop(models,configurations,levels,parameters.rejections,tasks,parameters.pikParam,path);
    if(!solved && moving) {
      configurations = initial; path->clear();
      global_inverse_kinematics_solver::GIKParam gik = parameters.gikParam;
      const ContactCandidate& candidate = moveContact.c1.isStatic ? moveContact.c2 : moveContact.c1;
      gik.projectionModelIndex = getModelIndex(candidate,models);
      gik.projectionJointId = getJointId(candidate,*models[gik.projectionModelIndex]);
      gik.projectionLocalPose = getJointLocalPose(candidate,*models[gik.projectionModelIndex]);
      const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > hard{levels[0],levels[1]};
      solved = global_inverse_kinematics_solver::solveGIK(models,configurations,hard,levels[2],parameters.nominals,parameters.rejections,gik,path);
      if(solved && !path->empty()) configurations = path->back();
    }
    if(!solved) { configurations = initial; return false; }
    if(path->empty()) path->push_back(configurations);
    postState.transition.insert(postState.transition.end(),path->begin(),path->end());
    return true;
  }
  bool ContactPlanner::checkTransitionImpl(const ContactState& preState, ContactState& postState, const GLMPParam& parameters) {
    const std::vector<std::shared_ptr<pinocchio::Model> > models = parameters.getModels();
    if(preState.frame.size() != models.size()) throw std::invalid_argument("ContactPlanner: invalid state frame");
    if(!this->checkObjectFeasibility(preState,parameters) || !this->checkObjectFeasibility(postState,parameters)) return false;
    for(std::size_t i = 0; i < models.size(); ++i)
      if(preState.frame[i].size() != models[i]->nq) throw std::invalid_argument("ContactPlanner: invalid configuration size");
    std::vector<Eigen::VectorXd> configurations = preState.frame;
    ContactState supportState = preState;
    std::vector<std::shared_ptr<pinocchio::Data> > data = ik_constraint::IKConstraint::createDataVector(models);
    for(std::size_t i = 0; i < models.size(); ++i) pinocchio::forwardKinematics(*models[i],*data[i],configurations[i]);
    for(std::size_t i = 0; i < supportState.contacts.size(); ++i) {
      fixContactYaw(supportState.contacts[i],models,data);
      for(std::size_t j = 0; j < postState.contacts.size(); ++j) {
        if(postState.contacts[j] == supportState.contacts[i]) {
          postState.contacts[j].yaw = supportState.contacts[i].yaw;
          postState.contacts[j].yawFixed = true;
        }
      }
    }
    postState.transition.clear();
    if(postState.contacts.size() == preState.contacts.size()+1) {
      Contact& move = postState.contacts.back();
      if(!this->solveContactIK(supportState,move,postState,IKState::ATTACH_PRE,configurations,parameters) ||
         !this->solveContactIK(supportState,move,postState,IKState::ATTACH_FIXED,configurations,parameters)) return false;
    } else if(postState.contacts.size()+1 == preState.contacts.size()) {
      Contact move; bool found = false;
      for(std::size_t i = 0; i < supportState.contacts.size(); ++i) {
        if(std::find(postState.contacts.begin(),postState.contacts.end(),supportState.contacts[i]) == postState.contacts.end()) {
          move = supportState.contacts[i]; found = true; break;
        }
      }
      if(!found || !this->solveContactIK(postState,move,postState,IKState::DETACH_FIXED,configurations,parameters)) return false;
    } else if(preState == postState) {
      Contact noMove;
      if(!this->solveContactIK(supportState,noMove,postState,IKState::SLIDE,configurations,parameters)) return false;
    } else return false;
    postState.frame = configurations;
    for(std::size_t i = 0; i < models.size(); ++i) pinocchio::forwardKinematics(*models[i],*data[i],configurations[i]);
    for(std::size_t i = 0; i < postState.contacts.size(); ++i) fixContactYaw(postState.contacts[i],models,data);
    return true;
  }
  void generateCandidateMarkers(mujoco_viewer::Viewer& viewer,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::shared_ptr<ContactCandidate> >& candidates) {
    std::vector<std::shared_ptr<pinocchio::Data> > data = ik_constraint::IKConstraint::createDataVector(models);
    for(std::size_t i = 0; i < models.size(); ++i) pinocchio::forwardKinematics(*models[i],*data[i],configurations[i]);
    const std::array<float,4> colors[3] = {{{1,0,0,1}},{{0,1,0,1}},{{0,0,1,1}}};
    for(std::size_t i = 0; i < candidates.size(); ++i) {
      const pinocchio::SE3 pose = getWorldPose(*candidates[i],models,data);
      const Eigen::Vector3d start = pose.translation();
      for(int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d end = start+0.04*pose.rotation().col(axis);
        mujoco_viewer::appendLine(viewer,{start.x(),start.y(),start.z()},{end.x(),end.y(),end.z()},2.0,colors[axis]);
      }
    }
  }
}
std::ostream& operator<<(std::ostream& stream, const graph_locomanip_planner::ContactState& state) {
  for(std::size_t i = 0; i < state.contacts.size(); ++i) {
    const graph_locomanip_planner::Contact& contact = state.contacts[i];
    stream << contact.c1.bodyName << "/" << contact.c1.linkName;
    if(!contact.c1.name.empty()) stream << "[" << contact.c1.name << "]";
    stream << " -> " << contact.c2.bodyName << "/" << contact.c2.linkName;
    if(!contact.c2.name.empty()) stream << "[" << contact.c2.name << "]";
    stream << " at " << contact.c2.localPose.translation().transpose() << '\n';
  }
  return stream;
}
