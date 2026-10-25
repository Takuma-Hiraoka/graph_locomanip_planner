#pragma once
#include <graph_locomanip_planner/contact_graph.h>
#include <pinocchio/multibody/geometry.hpp>
#include <mujoco_esdf/mujoco_esdf.h>
namespace graph_locomanip_planner_sample {
  struct SampleModel {
    std::string robotDirectory;
    std::string robotUrdf;
    std::shared_ptr<pinocchio::GeometryModel> robotGeometry;
    std::shared_ptr<pinocchio::GeometryModel> objectGeometry;
    std::shared_ptr<voxblox::EsdfMap> esdfMap;
    std::shared_ptr<mjModel> viewerModel;
  };
  void generateG1(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample);
  void generateStepWorld(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample);
  void generateManipulationWorld(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample);
  void addRobotCollisionConstraints(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample);
  void setViewerConfiguration(mujoco_viewer::Viewer& viewer, const pinocchio::Model& model, const Eigen::VectorXd& q,
    const std::string& rootJointName = "root_joint");
  void setViewerConfigurations(mujoco_viewer::Viewer& viewer,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models, const std::vector<Eigen::VectorXd>& configurations);
  void drawSupport(mujoco_viewer::Viewer& viewer, const graph_locomanip_planner::ContactPlanner& planner,
    const graph_locomanip_planner::ContactState& state, const std::vector<Eigen::VectorXd>& configurations);
}
