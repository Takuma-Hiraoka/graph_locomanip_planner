#include <graph_locomanip_planner_sample/sample_common.h>
#include <ik_constraint_esdf/EsdfCollisionConstraint.h>
#include <ik_constraint_vclip/VclipCollisionConstraint.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <set>
#include <stdexcept>

namespace graph_locomanip_planner_sample {
  namespace {
    mjsGeom* addBox(mjSpec* spec, const std::string& name,
      const Eigen::Vector3d& position, const Eigen::Vector3d& size) {
      mjsGeom* geom = mjs_addGeom(mjs_findBody(spec,"world"),nullptr);
      mjs_setName(geom->element,name.c_str()); geom->type = mjGEOM_BOX;
      for(int axis = 0; axis < 3; ++axis) { geom->pos[axis] = position[axis]; geom->size[axis] = size[axis]/2.0; }
      geom->rgba[0] = 0.5f; geom->rgba[1] = 0.6f; geom->rgba[2] = 0.7f; geom->rgba[3] = 1.0f;
      return geom;
    }
  }
  void generateStepWorld(graph_locomanip_planner::ContactPlanner::GLMPParam& param, SampleModel& sample) {
    char error[1024] = {};
    std::shared_ptr<mjSpec> spec(mj_parseXML(sample.robotUrdf.c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
    if(!spec) throw std::runtime_error(error);
    // URDF roots are fixed by default. Match the Pinocchio free-flyer root.
    spec->compiler.fusestatic = false;
    mjsBody* pelvis = mjs_findBody(spec.get(),"pelvis");
    if(!pelvis) throw std::runtime_error("G1 pelvis body was not found in the URDF");
    mjsJoint* root = mjs_addFreeJoint(pelvis);
    mjs_setName(root->element,"root_joint");
    // Keep the support below the hands in the initial configuration.
    const Eigen::Vector3d handSupportPosition(0.38,0,0.50);
    const Eigen::Vector3d handSupportSize(0.10,0.7,0.05);
    const double handSupportTop = handSupportPosition.z() + handSupportSize.z() / 2.0;
    const std::vector<const mjsGeom*> obstacleGeometries{
      addBox(spec.get(),"planner_floor",Eigen::Vector3d(0,0,-0.05),Eigen::Vector3d(3,2,0.1)),
      addBox(spec.get(),"step",Eigen::Vector3d(0.27,0,0.04),Eigen::Vector3d(0.24,0.7,0.08)),
      addBox(spec.get(),"hand_support",handSupportPosition,handSupportSize)};
    // Build once from the same obstacle geometries that the viewer displays.
    sample.esdfMap = mujoco_esdf::createEsdfMap(obstacleGeometries,
      Eigen::Vector3d(-1.8,-1.3,-0.3), Eigen::Vector3d(1.8,1.3,2.2), 0.03, 16);
    const std::shared_ptr<pinocchio::Model>& model = param.robot;
    pinocchio::Data data(*model); pinocchio::forwardKinematics(*model,data,param.currentContactState->frame[0]);
    for(std::size_t i = 0; i < param.contactDynamicCandidates.size(); ++i) {
      const graph_locomanip_planner::ContactCandidate& dynamic = *param.contactDynamicCandidates[i];
      const bool hand = dynamic.linkName == "L_hand_base_link" || dynamic.linkName == "R_hand_base_link";
      const bool left = dynamic.linkName.find("left") != std::string::npos || dynamic.linkName == "L_hand_base_link";
      const Eigen::Vector3d target = hand ? Eigen::Vector3d(handSupportPosition.x(),left ? 0.20 : -0.20,handSupportTop) :
        Eigen::Vector3d(0.27,left ? 0.12 : -0.12,0.08);
      std::shared_ptr<graph_locomanip_planner::ContactCandidate> candidate =
        std::make_shared<graph_locomanip_planner::ContactCandidate>("floor2",hand ? "hand_support" : "step",
          pinocchio::SE3(Eigen::Matrix3d::Identity(),target));
      if(hand) candidate->weight.tail<3>() = Eigen::Vector3d(1,1,0);
      candidate->allowedDynamicLinks.push_back(dynamic.linkName);
      param.contactStaticCandidates.push_back(candidate);
      param.goalContactState->contacts.push_back(graph_locomanip_planner::Contact(dynamic,*candidate));
    }
    std::set<pinocchio::JointIndex> ids;
    for(std::size_t i = 0; i < sample.robotGeometry->geometryObjects.size(); ++i) {
      const pinocchio::GeometryObject& geometry = sample.robotGeometry->geometryObjects[i];
      if(geometry.geometry && !geometry.disableCollision &&
         geometry.parentJoint > 0 && geometry.parentJoint < model->njoints)
        ids.insert(geometry.parentJoint);
    }
    for(std::set<pinocchio::JointIndex>::const_iterator id = ids.begin(); id != ids.end(); ++id) {
      std::shared_ptr<ik_constraint_esdf::EsdfCollisionConstraint> collision =
        std::make_shared<ik_constraint_esdf::EsdfCollisionConstraint>(model,sample.robotGeometry,*id);
      collision->setField(sample.esdfMap);
      collision->setResolution(0.04);
      collision->setMinDistance(-0.03);
      collision->setMaxDistance(0.5);
      collision->setTolerance(0.005); collision->setPrecision(1e-3); collision->setIgnoreDistance(0.1);
      param.constraints.push_back(collision);
    }
    // Exclude the same joint and its parent, whose shapes intentionally touch.
    for(std::set<pinocchio::JointIndex>::const_iterator a = ids.begin(); a != ids.end(); ++a) {
      for(std::set<pinocchio::JointIndex>::const_iterator b = ids.upper_bound(*a); b != ids.end(); ++b) {
        if(model->parents[*a] == *b || model->parents[*b] == *a) continue;
        std::shared_ptr<ik_constraint_vclip::VclipCollisionConstraint> collision =
          std::make_shared<ik_constraint_vclip::VclipCollisionConstraint>(model,sample.robotGeometry,*a,model,sample.robotGeometry,*b);
        collision->setTolerance(0.001); collision->setPrecision(1e-3); collision->setIgnoreDistance(0.05);
        param.constraints.push_back(collision);
      }
    }
    sample.viewerModel = std::shared_ptr<mjModel>(mj_compile(spec.get(),nullptr),mj_deleteModel);
    if(!sample.viewerModel) throw std::runtime_error(mjs_getError(spec.get()));
  }
}
