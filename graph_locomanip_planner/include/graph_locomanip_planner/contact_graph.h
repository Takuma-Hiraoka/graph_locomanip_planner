#pragma once
#include <graph_search/graph.h>
#include <graph_locomanip_planner/contact_node.h>
#include <graph_locomanip_planner/util.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>
#include <ik_constraint_sp/SPConstraint.h>
#include <utility>

namespace graph_locomanip_planner {
  struct ObjectContactRequirement {
    std::shared_ptr<const pinocchio::Model> model;
    // Each pair is an alternative grasp: both object sites must touch robot sites.
    std::vector<std::pair<ContactCandidate,ContactCandidate> > liftContactPairs;
    // Only these object sites may support the object on a static environment.
    std::vector<ContactCandidate> supportCandidates;
  };
  bool isObjectGrasped(const ContactState& state, const ObjectContactRequirement& requirement,
    const std::string& robotName);
  bool isObjectStaticallySupported(const ContactState& state, const ObjectContactRequirement& requirement);

  class ContactPlanner : public graph_search::Planner {
  public:
    class GLMPParam {
    public:
      GLMPParam();
      std::shared_ptr<pinocchio::Model> robot;
      std::vector<std::shared_ptr<pinocchio::Model> > object;
      // Exactly one entry for each object, matched by the model shared pointer.
      std::vector<ObjectContactRequirement> objectContactRequirements;
      // Settings template; each IK phase builds a private copy with current
      // robot/static support polygons and robot + unsupported grasped objects.
      std::shared_ptr<ik_constraint_sp::SPConstraint> spConstraint;
      // IK/GIK configurations and ContactState frames use [robot, object...].
      std::vector<std::shared_ptr<pinocchio::Model> > getModels() const;
      std::vector<std::shared_ptr<ik_constraint::IKConstraint> > constraints, rejections, nominals;
      std::shared_ptr<ContactState> currentContactState, goalContactState;
      std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates, contactStaticCandidates;
      prioritized_inverse_kinematics_solver::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
      std::shared_ptr<mujoco_viewer::Viewer> viewer = nullptr;
      double addCandidateDistance = 1.5;
      double contactClearance = 0.03;
      double contactPrecision = 1e-3;
    };
    class ContactTransitionCheckParam : public graph_search::Planner::TransitionCheckParam {
    public:
      ContactState preState, postState;
      GLMPParam param;
    };
    GLMPParam param;
    bool solve();
    bool checkObjectFeasibility(const ContactState& state, const GLMPParam& parameters) const;
    std::shared_ptr<ik_constraint_sp::SPConstraint> createSPConstraint(
      const ContactState& supportState, const GLMPParam& parameters) const;
    void goalPath(std::vector<ContactState>& path);
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
      std::shared_ptr<graph_search::Node> node) override;
    void postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
      std::shared_ptr<graph_search::Node> node) override;
    bool checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(
      std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    void calcHeuristic(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam,
      std::shared_ptr<graph_search::Node> node) override;
    void addNodes2Graph(std::vector<std::shared_ptr<graph_search::Node> >& nodes) override;
    bool checkTransitionImpl(const ContactState& preState, ContactState& postState, const GLMPParam& parameters);
    bool solveContactIK(const ContactState& supportState, Contact& moveContact, ContactState& postState,
      const IKState& state, std::vector<Eigen::VectorXd>& configurations, const GLMPParam& parameters);
  };
}
