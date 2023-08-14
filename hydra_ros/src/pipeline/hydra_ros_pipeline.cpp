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
#include "hydra_ros/pipeline/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/hydra_config.h>
#include <hydra/loop_closure/loop_closure_module.h>

#include <memory>

#include "hydra_ros/pipeline/ros_backend.h"
#include "hydra_ros/pipeline/ros_backend_publisher.h"
#include "hydra_ros/pipeline/ros_frontend.h"
#include "hydra_ros/pipeline/ros_frontend_publisher.h"
#include "hydra_ros/pipeline/ros_lcd_registration.h"
#include "hydra_ros/pipeline/ros_reconstruction.h"
#include "hydra_ros/visualizer/reconstruction_visualizer.h"

namespace hydra {

void declare_config(HydraRosConfig& conf) {
  using namespace config;
  name("HydraRosConfig");
  field(conf.enable_lcd, "enable_lcd");
  field(conf.use_ros_backend, "use_ros_backend");
  field(conf.do_reconstruction, "do_reconstruction");
  field(conf.enable_frontend_output, "enable_frontend_output");
  field(conf.visualize_reconstruction, "visualize_reconstruction");
  field(conf.reconstruction_visualizer_namespace,
        "reconstruction_visualizer_namespace");
}

HydraRosPipeline::HydraRosPipeline(const HydraRosConfig& config,
                                   const ros::NodeHandle& node_handle,
                                   int robot_id,
                                   const LogSetup::Ptr& log_setup)
    : HydraPipeline(robot_id, log_setup), config_(config), nh_(node_handle) {
  const auto label_space =
      config::checkValid(config::fromRos<hydra::LabelSpaceConfig>(nh_));
  VLOG(1) << "Loaded label space:" << std::endl << config::toString(label_space);
  HydraConfig::instance().setLabelSpaceConfig(label_space);

  initFrontend();

  if (config_.do_reconstruction) {
    initReconstruction();
  }

  initBackend();

  if (config_.enable_lcd) {
    initLCD();
  }
}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::initFrontend() {
  // explict type for shared_ptr
  FrontendModule::Ptr frontend =
      config::createFromROS<FrontendModule>(ros::NodeHandle(nh_, "frontend"),
                                            prefix_,
                                            frontend_dsg_,
                                            shared_state_,
                                            log_setup_);
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
}

void HydraRosPipeline::initBackend() {
  // explict type for shared_ptr
  BackendModule::Ptr backend =
      config::createFromROS<BackendModule>(ros::NodeHandle(nh_, "backend"),
                                           prefix_,
                                           frontend_dsg_,
                                           backend_dsg_,
                                           shared_state_,
                                           log_setup_);

  modules_["backend"] = backend;
  if (!modules_.at("backend")) {
    LOG(FATAL) << "Failed to construct backend!";
  }

  auto backend_publisher =
      std::make_shared<RosBackendPublisher>(nh_, backend->config(), prefix_);
  modules_["backend_publisher"] = backend_publisher;
  backend->addOutputCallback(std::bind(&RosBackendPublisher::publish,
                                       backend_publisher.get(),
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));
}

void HydraRosPipeline::initReconstruction() {
  auto conf = config::checkValid(config::fromRos<RosReconstructionConfig>(nh_));
  if (!loadReconstructionExtrinsics(conf)) {
    LOG(ERROR) << "Could not load extrinsics! Falling back to parsed extrinsics";
  }

  InputQueue<ReconstructionOutput::Ptr>::Ptr frontend_queue;
  if (modules_.count("frontend")) {
    auto frontend = std::dynamic_pointer_cast<FrontendModule>(modules_.at("frontend"));
    if (frontend) {
      frontend_queue = frontend->getQueue();
    } else {
      LOG(ERROR) << "Invalid frontend module: disabling reconstruction output queue";
    }
  }

  const auto reconstruction =
      std::make_shared<RosReconstruction>(conf, nh_, prefix_, frontend_queue);
  modules_["reconstruction"] = reconstruction;

  if (config_.visualize_reconstruction) {
    const auto viz = std::make_shared<ReconstructionVisualizer>(
        config_.reconstruction_visualizer_namespace);
    reconstruction->addVisualizationCallback(
        std::bind(&ReconstructionVisualizer::visualize,
                  viz.get(),
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3));
    modules_["reconstruction_visualizer"] = viz;
  }

  // TODO(nathan) publish reconstruction output
}

void HydraRosPipeline::initLCD() {
  auto lcd_config = config::fromRos<LoopClosureConfig>(nh_);
  lcd_config.detector.num_semantic_classes = HydraConfig::instance().getTotalLabels();
  VLOG(1) << "Number of classes for LCD: " << lcd_config.detector.num_semantic_classes;
  config::checkValid(lcd_config);

  shared_state_->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
  auto lcd = std::make_shared<LoopClosureModule>(
      prefix_, lcd_config, frontend_dsg_, shared_state_);
  modules_["lcd"] = lcd;

  bow_sub_ = nh_.subscribe("bow_vectors", 100, &HydraRosPipeline::bowCallback, this);
  if (lcd_config.detector.enable_agent_registration) {
    lcd->getDetector().setRegistrationSolver(0,
                                             std::make_unique<lcd::DsgAgentSolver>());
  }
}

void HydraRosPipeline::bowCallback(const pose_graph_tools::BowQueries::ConstPtr& msg) {
  for (const auto& query : msg->queries) {
    shared_state_->visual_lcd_queue.push(
        pose_graph_tools::BowQuery::ConstPtr(new pose_graph_tools::BowQuery(query)));
  }
}

}  // namespace hydra
