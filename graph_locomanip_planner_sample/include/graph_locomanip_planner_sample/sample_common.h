#pragma once
#include <graph_locomanip_planner/contact_graph.h>
#include <pinocchio/multibody/geometry.hpp>
#include <mujoco_esdf/mujoco_esdf.h>
namespace graph_locomanip_planner_sample {
  struct SampleModel {
    std::string robotDirectory;
    std::string robotUrdf;
    std::shared_ptr<pinocchio::GeometryModel> robotGeometry;
    std::shared_ptr<voxblox::EsdfMap> esdfMap;
    std::shared_ptr<mjModel> viewerModel;
  };
  void generateG1(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample);
  void generateStepWorld(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample);
  void setViewerConfiguration(mujoco_viewer::Viewer& viewer, const pinocchio::Model& model, const Eigen::VectorXd& q);
}
