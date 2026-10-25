#include <graph_locomanip_planner_sample/sample_common.h>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
  try {
  graph_locomanip_planner::ContactPlanner planner;
  graph_locomanip_planner_sample::SampleModel sample;
  graph_locomanip_planner_sample::generateG1(planner.param,sample);
  graph_locomanip_planner_sample::generateStepWorld(planner.param,sample);
  const std::vector<std::shared_ptr<pinocchio::Model> > models = planner.param.getModels();
  planner.debugLevel() = 1;
  planner.threads() = 30;
  planner.maxExtendNum() = 200;
  if(argc > 1 && std::string(argv[1]) == "--headless") {
    const bool solved = planner.solve();
    std::vector<graph_locomanip_planner::ContactState> path; planner.goalPath(path);
    std::cout << "Contact planning " << (solved ? "succeeded" : "failed") << ": " << path.size() << " states" << std::endl;
    return solved ? 0 : 1;
  }
  std::shared_ptr<mujoco_viewer::Viewer> viewer = std::make_shared<mujoco_viewer::Viewer>();
  planner.param.viewer = viewer; viewer->viewModel(sample.viewerModel);
  viewer->option()->geomgroup[0] = 1; viewer->option()->geomgroup[1] = 1;
  viewer->camera()->distance = 2.5; viewer->camera()->lookat[2] = 0.6;
  graph_locomanip_planner_sample::setViewerConfiguration(*viewer,*planner.param.robot,planner.param.currentContactState->frame[0]);
  std::future<bool> result = std::async(std::launch::async,[&planner]() { return planner.solve(); });
  while(result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    if(viewer->isOpen()) {
      viewer->updateScene();
      graph_locomanip_planner_sample::drawSupport(*viewer,planner,*planner.param.currentContactState,planner.param.currentContactState->frame);
      graph_locomanip_planner::generateCandidateMarkers(*viewer,models,
        planner.param.currentContactState->frame,planner.param.contactStaticCandidates);
      viewer->drawScene(); viewer->pollEvents();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  if(!result.get()) { std::cerr << "Contact planning failed" << std::endl; return 1; }
  std::vector<graph_locomanip_planner::ContactState> path; planner.goalPath(path);
  std::cout << "Contact planning succeeded: " << path.size() << " states" << std::endl;
  while(viewer->isOpen()) {
    for(std::size_t state = 0; state < path.size() && viewer->isOpen(); ++state) {
      std::cout << path[state];
      const std::size_t count = path[state].transition.size();
      for(std::size_t frame = 0; frame < count && viewer->isOpen(); ++frame) {
        graph_locomanip_planner_sample::setViewerConfiguration(*viewer,*planner.param.robot,path[state].transition[frame][0]);
        viewer->updateScene();
        graph_locomanip_planner_sample::drawSupport(*viewer,planner,path[state],path[state].transition[frame]);
        graph_locomanip_planner::generateCandidateMarkers(*viewer,models,path[state].transition[frame],planner.param.contactStaticCandidates);
        viewer->drawScene(); viewer->pollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }
  return 0;
  } catch(const std::exception& error) {
    std::cerr << "sample0: " << error.what() << std::endl;
    return 1;
  }
}
