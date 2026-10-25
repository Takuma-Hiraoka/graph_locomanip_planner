#include <graph_locomanip_planner_sample/sample_common.h>
#include <ik_constraint_esdf/EsdfCollisionConstraint.h>
#include <ik_constraint_vclip/VclipCollisionConstraint.h>
#include <pinocchio_primitive_models/pinocchio_primitive_models.h>
#include <pinocchio_mujoco_converter/pinocchio_mujoco_converter.h>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <set>
#include <cmath>
#include <stdexcept>

namespace graph_locomanip_planner_sample {
  namespace {
    mjsGeom* addSupport(mjSpec& spec, const std::string& name,
      const Eigen::Vector3d& position, const Eigen::Vector3d& size) {
      mjsGeom* geom = mjs_addGeom(mjs_findBody(&spec,"world"),nullptr);
      mjs_setName(geom->element,name.c_str());
      geom->type = mjGEOM_BOX;
      for(int axis = 0; axis < 3; ++axis) {
        geom->pos[axis] = position[axis]; geom->size[axis] = size[axis]/2.0;
      }
      geom->rgba[0] = 0.6f; geom->rgba[1] = 0.6f; geom->rgba[2] = 0.65f; geom->rgba[3] = 1.0f;
      return geom;
    }
  }
  void generateManipulationWorld(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample) {
    char error[1024] = {};
    std::shared_ptr<mjSpec> spec(mj_parseXML(sample.robotUrdf.c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
    if(!spec) throw std::runtime_error(error);
    spec->compiler.fusestatic = false;
    mjsBody* pelvis = mjs_findBody(spec.get(),"pelvis");
    if(!pelvis) throw std::runtime_error("G1 pelvis body is missing");
    mjsJoint* root = mjs_addFreeJoint(pelvis);
    mjs_setName(root->element,"root_joint");
    pinocchio::Data data(*param.robot);
    pinocchio::forwardKinematics(*param.robot,data,param.currentContactState->frame[0]);
    std::vector<graph_locomanip_planner::ContactCandidate> hands;
    for(std::size_t i = 0; i < param.contactDynamicCandidates.size(); ++i) {
      graph_locomanip_planner::ContactCandidate& candidate = *param.contactDynamicCandidates[i];
      candidate.allowStaticContact = false;
      if(candidate.linkName == "L_hand_base_link" || candidate.linkName == "R_hand_base_link") {
        candidate.allowedDynamicLinks = {"base_joint"};
        hands.push_back(candidate);
      } else candidate.allowDynamicContact = false;
    }
    if(hands.size() != 2) throw std::runtime_error("Expected two G1 hand contact sites");
    std::vector<std::shared_ptr<pinocchio::Model> > robotModels{param.robot};
    std::vector<std::shared_ptr<pinocchio::Data> > robotData{std::make_shared<pinocchio::Data>(data)};
    const pinocchio::SE3 leftPose = graph_locomanip_planner::getWorldPose(hands[0],robotModels,robotData);
    const pinocchio::SE3 rightPose = graph_locomanip_planner::getWorldPose(hands[1],robotModels,robotData);
    const Eigen::Vector3d initialPosition = 0.5*(leftPose.translation()+rightPose.translation());
    const Eigen::Vector3d size(0.16,std::abs(leftPose.translation().y()-rightPose.translation().y()),0.16);
    const double mass = 5.0;
    std::shared_ptr<pinocchio::Model> box = std::make_shared<pinocchio::Model>();
    sample.objectGeometry = std::make_shared<pinocchio::GeometryModel>();
    pinocchio_primitive_models::createBox(size,mass,*box,*sample.objectGeometry);
    box->name = "box";
    param.object.push_back(box);
    Eigen::VectorXd boxQ = pinocchio::neutral(*box);
    boxQ.head<3>() = initialPosition;
    param.currentContactState->frame.push_back(boxQ);
    pinocchio_mujoco_converter::createBox(size,mass,*spec,box->name);

    const double initialTop = initialPosition.z()-0.5*size.z();
    const Eigen::Vector3d targetPosition = initialPosition+Eigen::Vector3d(0.18,0,0.10);
    const double tableTop = targetPosition.z()-0.5*size.z();
    const std::vector<const mjsGeom*> environment{
      addSupport(*spec,"floor",Eigen::Vector3d(0,0,-0.05),Eigen::Vector3d(3,2,0.1)),
      addSupport(*spec,"initial_support",Eigen::Vector3d(initialPosition.x(),0,0.5*initialTop),Eigen::Vector3d(0.12,0.5*size.y(),initialTop)),
      addSupport(*spec,"table",Eigen::Vector3d(targetPosition.x(),0,0.5*tableTop),Eigen::Vector3d(0.16,0.5*size.y(),tableTop))};
    sample.esdfMap = mujoco_esdf::createEsdfMap(environment,
      Eigen::Vector3d(-1.8,-1.3,-0.3),Eigen::Vector3d(1.8,1.3,2.2),0.03,16);

    graph_locomanip_planner::ObjectContactRequirement requirement;
    requirement.model = box;
    std::vector<graph_locomanip_planner::ContactCandidate> grips;
    for(int side = 0; side < 2; ++side) {
      const double sign = side == 0 ? 1.0 : -1.0;
      Eigen::Matrix3d rotation;
      rotation.col(0) = Eigen::Vector3d::UnitX();
      rotation.col(1) = -sign*Eigen::Vector3d::UnitZ();
      rotation.col(2) = sign*Eigen::Vector3d::UnitY();
      graph_locomanip_planner::ContactCandidate grip("box","base_joint",
        pinocchio::SE3(rotation,Eigen::Vector3d(0,sign*size.y()/2.0,0)),false);
      grip.name = side == 0 ? "left_grip" : "right_grip";
      grip.allowStaticContact = false;
      grip.allowedDynamicLinks = {hands[side].linkName};
      grips.push_back(grip);
      param.contactDynamicCandidates.push_back(std::make_shared<graph_locomanip_planner::ContactCandidate>(grip));
    }
    requirement.liftContactPairs.push_back(std::make_pair(grips[0],grips[1]));
    graph_locomanip_planner::ContactCandidate bottom("box","base_joint",
      pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,-size.z()/2.0)),false);
    bottom.name = "bottom";
    bottom.allowDynamicContact = false;
    bottom.allowedStaticLinks = {"initial_support","table"};
    requirement.supportCandidates.push_back(bottom);
    param.objectContactRequirements.push_back(requirement);
    param.contactDynamicCandidates.push_back(std::make_shared<graph_locomanip_planner::ContactCandidate>(bottom));
    graph_locomanip_planner::ContactCandidate initialSupport("environment","initial_support",
      pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(initialPosition.x(),0,initialTop)));
    graph_locomanip_planner::ContactCandidate table("environment","table",
      pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(targetPosition.x(),0,tableTop)));
    initialSupport.allowedDynamicLinks = {"base_joint"};
    table.allowedDynamicLinks = {"base_joint"};
    param.contactStaticCandidates.push_back(std::make_shared<graph_locomanip_planner::ContactCandidate>(initialSupport));
    param.contactStaticCandidates.push_back(std::make_shared<graph_locomanip_planner::ContactCandidate>(table));
    param.goalContactState->contacts = param.currentContactState->contacts;
    param.currentContactState->contacts.push_back(graph_locomanip_planner::Contact(bottom,initialSupport));
    param.goalContactState->contacts.push_back(graph_locomanip_planner::Contact(bottom,table));
    addRobotCollisionConstraints(param,sample);
    std::set<pinocchio::JointIndex> joints;
    for(const pinocchio::GeometryObject& geometry : sample.robotGeometry->geometryObjects)
      if(geometry.geometry && !geometry.disableCollision && geometry.parentJoint > 0) joints.insert(geometry.parentJoint);
    for(std::set<pinocchio::JointIndex>::const_iterator joint = joints.begin(); joint != joints.end(); ++joint) {
      std::shared_ptr<ik_constraint_vclip::VclipCollisionConstraint> collision =
        std::make_shared<ik_constraint_vclip::VclipCollisionConstraint>(param.robot,sample.robotGeometry,*joint,box,sample.objectGeometry,1);
      collision->setTolerance(0.001); collision->setPrecision(1e-3); collision->setIgnoreDistance(0.05);
      param.constraints.push_back(collision);
    }
    std::shared_ptr<ik_constraint_esdf::EsdfCollisionConstraint> boxCollision =
      std::make_shared<ik_constraint_esdf::EsdfCollisionConstraint>(box,sample.objectGeometry,1);
    boxCollision->setField(sample.esdfMap);
    boxCollision->setMinDistance(-0.03); boxCollision->setMaxDistance(0.5);
    boxCollision->setTolerance(0.005); boxCollision->setPrecision(1e-3); boxCollision->setIgnoreDistance(0.1);
    param.constraints.push_back(boxCollision);
    sample.viewerModel = std::shared_ptr<mjModel>(mj_compile(spec.get(),nullptr),mj_deleteModel);
    if(!sample.viewerModel) throw std::runtime_error(mjs_getError(spec.get()));
  }
}
