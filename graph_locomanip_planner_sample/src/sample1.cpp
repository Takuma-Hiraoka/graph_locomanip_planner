#include <graph_locomanip_planner_sample/sample_common.h>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char** argv) {
  try {
    bool headless = false;
    unsigned int threads = 1;
    for(int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);
      if(argument == "--headless") headless = true;
      else if(argument == "--threads" && i+1 < argc) threads = std::stoul(argv[++i]);
      else throw std::invalid_argument("Usage: sample1 [--headless] [--threads N]");
    }
    graph_locomanip_planner::ContactPlanner planner;
    graph_locomanip_planner_sample::SampleModel sample;
    graph_locomanip_planner_sample::generateG1(planner.param,sample);
    graph_locomanip_planner_sample::generateManipulationWorld(planner.param,sample);
    planner.debugLevel() = 1;
    planner.threads() = threads;
    planner.maxExtendNum() = 200;
    planner.param.pikParam.maxIteration = 200;
    const std::vector<std::shared_ptr<pinocchio::Model> > models = planner.param.getModels();
    std::shared_ptr<mujoco_viewer::Viewer> viewer;
    if(!headless) {
      viewer = std::make_shared<mujoco_viewer::Viewer>();
      viewer->viewModel(sample.viewerModel);
      viewer->camera()->distance = 2.5; viewer->camera()->lookat[2] = 0.6;
      graph_locomanip_planner_sample::setViewerConfigurations(*viewer,models,planner.param.currentContactState->frame);
    }
    std::future<bool> result = std::async(std::launch::async,[&planner]() { return planner.solve(); });
    while(result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      if(viewer && viewer->isOpen()) {
        viewer->updateScene();
        graph_locomanip_planner_sample::drawSupport(*viewer,planner,*planner.param.currentContactState,planner.param.currentContactState->frame);
        graph_locomanip_planner::generateCandidateMarkers(*viewer,models,
          planner.param.currentContactState->frame,planner.param.contactDynamicCandidates);
        graph_locomanip_planner::generateCandidateMarkers(*viewer,models,
          planner.param.currentContactState->frame,planner.param.contactStaticCandidates);
        viewer->drawScene(); viewer->pollEvents();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    if(!result.get()) { std::cerr << "Box manipulation planning failed" << std::endl; return 1; }
    std::vector<graph_locomanip_planner::ContactState> path; planner.goalPath(path);
    std::cout << "Box manipulation planning succeeded: " << path.size() << " states" << std::endl;
    bool carried = false;
    for(std::size_t i = 0; i < path.size(); ++i) {
      std::shared_ptr<ik_constraint_sp::SPConstraint> support = planner.createSPConstraint(path[i],planner.param);
      std::cout << "State " << i << ": COM models =";
      for(const std::shared_ptr<const pinocchio::Model>& model : support->getCOMModels()) std::cout << " " << model->name;
      std::cout << "\n" << path[i];
      if(support->getCOMModels().size() > 1) carried = true;
    }
    if(!carried) { std::cerr << "Path did not lift the box" << std::endl; return 1; }
    while(viewer && viewer->isOpen()) {
      for(std::size_t state = 0; state < path.size() && viewer->isOpen(); ++state) {
        for(std::size_t frame = 0; frame < path[state].transition.size() && viewer->isOpen(); ++frame) {
          const std::vector<Eigen::VectorXd>& configurations = path[state].transition[frame];
          graph_locomanip_planner_sample::setViewerConfigurations(*viewer,models,configurations);
          viewer->updateScene();
          graph_locomanip_planner_sample::drawSupport(*viewer,planner,path[state],configurations);
          graph_locomanip_planner::generateCandidateMarkers(*viewer,models,configurations,planner.param.contactStaticCandidates);
          viewer->drawScene(); viewer->pollEvents();
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
    }
    return 0;
  } catch(const std::exception& error) {
    std::cerr << "sample1: " << error.what() << std::endl;
    return 1;
  }
}
