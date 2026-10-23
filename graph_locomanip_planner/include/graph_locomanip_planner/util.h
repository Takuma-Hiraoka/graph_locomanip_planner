#pragma once
#include <graph_locomanip_planner/contact_state.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <memory>
#include <ostream>
namespace graph_locomanip_planner {
  enum class IKState { DETACH_FIXED, ATTACH_PRE, ATTACH_FIXED, ATTACH_SEARCH, SLIDE };
  std::size_t getModelIndex(const ContactCandidate& candidate,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models);
  pinocchio::JointIndex getJointId(const ContactCandidate& candidate, const pinocchio::Model& model);
  pinocchio::SE3 getJointLocalPose(const ContactCandidate& candidate, const pinocchio::Model& model);
  pinocchio::SE3 getWorldPose(const ContactCandidate& candidate,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data);
  void generateCandidateMarkers(mujoco_viewer::Viewer& viewer,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::shared_ptr<ContactCandidate> >& candidates);
}
std::ostream& operator<<(std::ostream& stream, const graph_locomanip_planner::ContactState& state);
