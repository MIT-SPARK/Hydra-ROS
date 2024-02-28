/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/common/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/dsg_types.h>
#include <hydra/common/hydra_config.h>
#include <hydra/frontend/frontend_module.h>
#include <hydra/loop_closure/loop_closure_module.h>

#include <memory>

#include "hydra_ros/backend/ros_backend.h"
#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/frontend/ros_frontend_publisher.h"
#include "hydra_ros/loop_closure/ros_lcd_registration.h"
#include "hydra_ros/reconstruction/ros_reconstruction.h"
#include "hydra_ros/visualizer/object_visualizer.h"
#include "hydra_ros/visualizer/places_visualizer.h"
#include "hydra_ros/visualizer/reconstruction_visualizer.h"

namespace hydra {

void declare_config(HydraRosConfig& conf) {
  using namespace config;
  name("HydraRosConfig");
  field(conf.use_ros_backend, "use_ros_backend");
  field(conf.enable_frontend_output, "enable_frontend_output");
  field(conf.visualize_objects, "visualize_objects");
  field(conf.visualize_places, "visualize_places");
  field(conf.places_visualizer_namespace, "places_visualizer_namespace");
  field(conf.visualize_reconstruction, "visualize_reconstruction");
  field(conf.reconstruction_visualizer_namespace,
        "reconstruction_visualizer_namespace");
}

HydraRosPipeline::HydraRosPipeline(const ros::NodeHandle& nh, int robot_id)
    : HydraPipeline(config::fromRos<PipelineConfig>(nh), robot_id),
      config_(config::checkValid(config::fromRos<HydraRosConfig>(nh))),
      nh_(nh) {}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  const auto& pipeline_config = HydraConfig::instance().getConfig();
  initFrontend();
  initBackend();

  if (pipeline_config.enable_reconstruction) {
    initReconstruction();
  }

  if (pipeline_config.enable_lcd) {
    initLCD();
  }
}

void HydraRosPipeline::initFrontend() {
  // explict type for shared_ptr
  FrontendModule::Ptr frontend =
      config::createFromROS<FrontendModule>(ros::NodeHandle(nh_, "frontend"),
                                            frontend_dsg_,
                                            shared_state_,
                                            HydraConfig::instance().getLogs());
  modules_["frontend"] = frontend;

  if (!config_.enable_frontend_output) {
    return;
  }

  if (!frontend) {
    LOG(FATAL) << "Frontend module required!";
    return;
  }

  auto frontend_publisher = std::make_shared<RosFrontendPublisher>(nh_);
  modules_["frontend_publisher"] = frontend_publisher;
  frontend->addOutputCallback(std::bind(&RosFrontendPublisher::publish,
                                        frontend_publisher.get(),
                                        std::placeholders::_1,
                                        std::placeholders::_2,
                                        std::placeholders::_3));

  if (config_.visualize_objects) {
    auto obj_conf = config::fromRos<ObjectVisualizerConfig>(nh_);
    const auto viz = std::make_shared<ObjectVisualizer>(obj_conf);
    frontend->addObjectVisualizationCallback(
        [&viz](const auto& cloud, const auto& indices, const auto& labels) {
          viz->visualize(cloud, indices, labels);
        });
    modules_["object_visualizer"] = viz;
  }

  if (config_.visualize_places) {
    const auto viz =
        std::make_shared<PlacesVisualizer>(config_.places_visualizer_namespace);
    frontend->addPlaceVisualizationCallback(std::bind(&PlacesVisualizer::visualize,
                                                      viz.get(),
                                                      std::placeholders::_1,
                                                      std::placeholders::_2,
                                                      std::placeholders::_3));
    modules_["places_visualizer"] = viz;
  }

  freespace_server_ = nh_.advertiseService(
      "query_freespace", &HydraRosPipeline::handleFreespaceSrv, this);
}

bool HydraRosPipeline::handleFreespaceSrv(hydra_msgs::QueryFreespace::Request& req,
                                          hydra_msgs::QueryFreespace::Response& res) {
  if (req.x.size() != req.y.size() || req.x.size() != req.z.size()) {
    return false;
  }

  if (req.x.empty()) {
    return true;
  }

  FrontendModule::PositionMatrix points(3, req.x.size());
  for (size_t i = 0; i < req.x.size(); ++i) {
    points(0, i) = req.x[i];
    points(1, i) = req.y[i];
    points(2, i) = req.z[i];
  }

  auto frontend = std::dynamic_pointer_cast<FrontendModule>(modules_.at("frontend"));
  const auto result = frontend->inFreespace(points, req.freespace_distance_m);
  for (const auto flag : result) {
    res.in_freespace.push_back(flag ? 1 : 0);
  }
  return true;
}

void HydraRosPipeline::initBackend() {
  // explict type for shared_ptr
  BackendModule::Ptr backend =
      config::createFromROS<BackendModule>(ros::NodeHandle(nh_, "backend"),
                                           frontend_dsg_,
                                           backend_dsg_,
                                           shared_state_,
                                           HydraConfig::instance().getLogs());

  modules_["backend"] = backend;
  if (!modules_.at("backend")) {
    LOG(FATAL) << "Failed to construct backend!";
  }

  auto frontend = std::dynamic_pointer_cast<FrontendModule>(modules_.at("frontend"));
  if (frontend->config().use_2d_places) {
    auto places_functor = std::make_shared<dsg_updates::Update2dPlacesFunctor>(
        backend->config().places2d_config);
    backend->setUpdateFunctor(DsgLayers::PLACES, places_functor);
  }

  auto backend_publisher =
      std::make_shared<RosBackendPublisher>(nh_, backend->config());
  modules_["backend_publisher"] = backend_publisher;
  backend->addOutputCallback(std::bind(&RosBackendPublisher::publish,
                                       backend_publisher.get(),
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));
}

void HydraRosPipeline::initReconstruction() {
  InputQueue<ReconstructionOutput::Ptr>::Ptr frontend_queue;
  if (modules_.count("frontend")) {
    auto frontend = std::dynamic_pointer_cast<FrontendModule>(modules_.at("frontend"));
    if (frontend) {
      frontend_queue = frontend->getQueue();
    } else {
      LOG(ERROR) << "Invalid frontend module: disabling reconstruction output queue";
    }
  }

  ReconstructionModule::Ptr mod = config::createFromROS<ReconstructionModule>(
      ros::NodeHandle(nh_, "reconstruction"), frontend_queue);
  modules_["reconstruction"] = mod;

  if (config_.visualize_reconstruction) {
    const auto viz = std::make_shared<ReconstructionVisualizer>(
        config_.reconstruction_visualizer_namespace);
    mod->addVisualizationCallback(std::bind(&ReconstructionVisualizer::visualize,
                                            viz.get(),
                                            std::placeholders::_1,
                                            std::placeholders::_2,
                                            std::placeholders::_3));
    modules_["reconstruction_visualizer"] = viz;
  }
}

void HydraRosPipeline::initLCD() {
  auto lcd_config = config::fromRos<LoopClosureConfig>(nh_);
  lcd_config.detector.num_semantic_classes = HydraConfig::instance().getTotalLabels();
  VLOG(1) << "Number of classes for LCD: " << lcd_config.detector.num_semantic_classes;
  config::checkValid(lcd_config);

  shared_state_->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
  auto lcd =
      std::make_shared<LoopClosureModule>(lcd_config, frontend_dsg_, shared_state_);
  modules_["lcd"] = lcd;

  bow_sub_ = nh_.subscribe("bow_vectors", 100, &HydraRosPipeline::bowCallback, this);
  if (lcd_config.detector.enable_agent_registration) {
    lcd->getDetector().setRegistrationSolver(0,
                                             std::make_unique<lcd::DsgAgentSolver>());
  }
}

void HydraRosPipeline::bowCallback(
    const pose_graph_tools_msgs::BowQueries::ConstPtr& msg) {
  for (const auto& query : msg->queries) {
    shared_state_->visual_lcd_queue.push(pose_graph_tools_msgs::BowQuery::ConstPtr(
        new pose_graph_tools_msgs::BowQuery(query)));
  }
}

}  // namespace hydra
