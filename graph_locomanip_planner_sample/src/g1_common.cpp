#include <graph_locomanip_planner_sample/sample_common.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/JointLimitConstraint.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio_model_builder/pinocchio_model_builder.h>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <stdexcept>

namespace graph_locomanip_planner_sample {
  void generateG1(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample) {
    if(sample.robotDirectory.empty())
      sample.robotDirectory = ament_index_cpp::get_package_share_directory("graph_locomanip_planner_sample") + "/unitree_ros/robots/g1_description";
    sample.robotUrdf = sample.robotDirectory + "/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf";
    const std::string& urdf = sample.robotUrdf;
    const std::vector<std::string> fingerJointNames{
      "thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint",
      "thumb_intermediate_joint", "thumb_distal_joint",
      "index_proximal_joint", "index_intermediate_joint",
      "middle_proximal_joint", "middle_intermediate_joint",
      "ring_proximal_joint", "ring_intermediate_joint",
      "pinky_proximal_joint", "pinky_intermediate_joint"};
    std::vector<std::string> fixedJointNames;
    for(const std::string& name : fingerJointNames) {
      fixedJointNames.push_back("L_" + name);
      fixedJointNames.push_back("R_" + name);
    }
    std::shared_ptr<pinocchio::Model> model = std::make_shared<pinocchio::Model>();
    pinocchio_model_builder::buildModel(*model,urdf,fixedJointNames);
    model->name = "G1";
    sample.robotGeometry = std::make_shared<pinocchio::GeometryModel>();
    // Build against the fixed-joint model: link frames retain the accumulated
    // fixed transforms, and buildGeom expresses each shape placement in its
    // parent joint frame (wrist_yaw_joint for the entire hand and fingers).
    pinocchio::urdf::buildGeom(*model,urdf,pinocchio::COLLISION,*sample.robotGeometry,std::vector<std::string>{sample.robotDirectory});
    // Ignore these wrist shapes in both self-collision and environment checks.
    for(pinocchio::GeometryObject& geometry : sample.robotGeometry->geometryObjects) {
      const std::string& linkName = model->frames[geometry.parentFrame].name;
      if(linkName == "left_wrist_roll_link" || linkName == "right_wrist_roll_link")
        geometry.disableCollision = true;
    }
    Eigen::VectorXd q = pinocchio::neutral(*model); q[2] = 0.793;
    const std::string sides[2] = {"left","right"};
    for(int side = 0; side < 2; ++side) {
      q[model->idx_qs[model->getJointId(sides[side]+"_hip_pitch_joint")]] = -0.2;
      q[model->idx_qs[model->getJointId(sides[side]+"_knee_joint")]] = 0.4;
      q[model->idx_qs[model->getJointId(sides[side]+"_ankle_pitch_joint")]] = -0.2;
      q[model->idx_qs[model->getJointId(sides[side]+"_shoulder_pitch_joint")]] = -0.3;
      q[model->idx_qs[model->getJointId(sides[side]+"_elbow_joint")]] = 0.8;
    }
    param.robot = model;
    param.object.clear();
    param.currentContactState = std::make_shared<graph_locomanip_planner::ContactState>();
    param.goalContactState = std::make_shared<graph_locomanip_planner::ContactState>();
    pinocchio::Data data(*model); pinocchio::forwardKinematics(*model,data,q);
    const pinocchio::JointIndex foot = model->getJointId("left_ankle_roll_joint");
    q[2] -= (data.oMi[foot]*pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.03,0,-0.035))).translation().z();
    pinocchio::forwardKinematics(*model,data,q);
    param.currentContactState->frame.push_back(q);
    // In each Inspire hand base frame, fingers extend along -Y and the
    // palm faces -X. Put the contact on the central palm surface, with
    // contact +X towards the fingers and contact +Z towards the hand back.
    Eigen::Matrix3d palmRotation;
    palmRotation.col(0) = -Eigen::Vector3d::UnitY();
    palmRotation.col(1) = -Eigen::Vector3d::UnitZ();
    palmRotation.col(2) = Eigen::Vector3d::UnitX();
    const pinocchio::SE3 palmPose(palmRotation,Eigen::Vector3d(-0.016,-0.09,0));
    for(int side = 0; side < 2; ++side) {
      std::shared_ptr<graph_locomanip_planner::ContactCandidate> leg =
        std::make_shared<graph_locomanip_planner::ContactCandidate>("G1",sides[side]+"_ankle_roll_link",
          pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.03,0,-0.035)),false);
      param.contactDynamicCandidates.push_back(leg);
      leg->supportPolygon = {Eigen::Vector2d(-0.10,-0.045),Eigen::Vector2d(-0.10,0.045),
        Eigen::Vector2d(0.10,0.045),Eigen::Vector2d(0.10,-0.045)};
      const pinocchio::SE3 pose = data.oMi[model->getJointId(sides[side]+"_ankle_roll_joint")] * leg->localPose;
      graph_locomanip_planner::ContactCandidate floor("floor","floor",pose);
      param.currentContactState->contacts.push_back(graph_locomanip_planner::Contact(*leg,floor));
      std::shared_ptr<graph_locomanip_planner::ContactCandidate> hand =
        std::make_shared<graph_locomanip_planner::ContactCandidate>("G1",side == 0 ? "L_hand_base_link" : "R_hand_base_link",
          palmPose,false);
      hand->weight.tail<3>() = Eigen::Vector3d(1,1,0);
      param.contactDynamicCandidates.push_back(hand);
    }
    param.spConstraint->setModels({model});
    for(pinocchio::JointIndex id = 1; id < model->njoints; ++id) {
      if(model->nqs[id] != 1 || model->nvs[id] != 1) continue;
      std::shared_ptr<ik_constraint::JointLimitConstraint> limit = std::make_shared<ik_constraint::JointLimitConstraint>(model);
      limit->setJointId(id); param.constraints.push_back(limit);
      std::shared_ptr<ik_constraint::JointAngleConstraint> nominal = std::make_shared<ik_constraint::JointAngleConstraint>(model);
      nominal->setAJointId(id); nominal->setBOffset(q[model->idx_qs[id]]); nominal->setPrecision(1e10);
      param.nominals.push_back(nominal);
    }
  }
  void setViewerConfiguration(mujoco_viewer::Viewer& viewer, const pinocchio::Model& model, const Eigen::VectorXd& q,
    const std::string& rootJointName) {
    // MuJoCo reads the original URDF and keeps the finger joints. Match the
    // zero angles at which pinocchio_model_builder fixes those joints.
    for(int id = 0; id < viewer.model()->njnt; ++id) {
      const char* name = mj_id2name(viewer.model(),mjOBJ_JOINT,id);
      if(!name) continue;
      const std::string jointName(name);
      if((jointName.rfind("L_",0) == 0 || jointName.rfind("R_",0) == 0) &&
         !model.existJointName(jointName) &&
         (viewer.model()->jnt_type[id] == mjJNT_HINGE || viewer.model()->jnt_type[id] == mjJNT_SLIDE))
        viewer.data()->qpos[viewer.model()->jnt_qposadr[id]] = 0.0;
    }
    for(pinocchio::JointIndex id = 1; id < model.njoints; ++id) {
      const Eigen::Index qi = model.idx_qs[id];
      if(model.nqs[id] == 7 && model.nvs[id] == 6) {
        const int joint = mj_name2id(viewer.model(),mjOBJ_JOINT,rootJointName.c_str());
        if(joint < 0 || viewer.model()->jnt_type[joint] != mjJNT_FREE)
          throw std::runtime_error("Viewer free joint missing: " + rootJointName);
        const int address = viewer.model()->jnt_qposadr[joint];
        for(int i = 0; i < 3; ++i) viewer.data()->qpos[address+i] = q[qi+i];
        viewer.data()->qpos[address+3] = q[qi+6];
        for(int i = 0; i < 3; ++i) viewer.data()->qpos[address+4+i] = q[qi+3+i];
      } else if(model.nqs[id] == 1) {
        const int joint = mj_name2id(viewer.model(),mjOBJ_JOINT,model.names[id].c_str());
        if(joint < 0) throw std::runtime_error("G1 viewer joint missing: " + model.names[id]);
        viewer.data()->qpos[viewer.model()->jnt_qposadr[joint]] = q[qi];
      }
    }
    mj_forward(viewer.model(),viewer.data());
  }
  void setViewerConfigurations(mujoco_viewer::Viewer& viewer,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models, const std::vector<Eigen::VectorXd>& configurations) {
    if(models.size() != configurations.size()) throw std::invalid_argument("Viewer model/configuration size mismatch");
    for(std::size_t i = 0; i < models.size(); ++i)
      setViewerConfiguration(viewer,*models[i],configurations[i],i == 0 ? "root_joint" : models[i]->name+"_base_joint");
  }
  void drawSupport(mujoco_viewer::Viewer& viewer, const graph_locomanip_planner::ContactPlanner& planner,
    const graph_locomanip_planner::ContactState& state, const std::vector<Eigen::VectorXd>& configurations) {
    const std::vector<std::shared_ptr<pinocchio::Model> > models = planner.param.getModels();
    std::shared_ptr<ik_constraint_sp::SPConstraint> support = planner.createSPConstraint(state,planner.param);
    if(!support) return;
    std::vector<std::shared_ptr<pinocchio::Data> > data = ik_constraint::IKConstraint::createDataVector(models);
    for(std::size_t i = 0; i < models.size(); ++i) {
      pinocchio::forwardKinematics(*models[i],*data[i],configurations[i]);
      pinocchio::centerOfMass(*models[i],*data[i],configurations[i]);
    }
    support->updateBounds(support->getConstDataVector(models,data));
    support->drawOn(viewer);
  }
}
