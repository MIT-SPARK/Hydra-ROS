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
#include "hydra_ros/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/backend/backend_module.h>
#include <hydra/backend/update_frontiers_functor.h>
#include <hydra/backend/update_surface_places_functor.h>
#include <hydra/common/dsg_types.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/frontend_module.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra/reconstruction/reconstruction_module.h>
#include <pose_graph_tools_ros/conversions.h>

#include <memory>

#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/frontend/ros_frontend_publisher.h"
#include "hydra_ros/loop_closure/ros_lcd_registration.h"
#include "hydra_ros/utils/bow_subscriber.h"

namespace hydra {

void declare_config(HydraRosConfig& conf) {
  using namespace config;
  name("HydraRosConfig");
  field(conf.enable_frontend_output, "enable_frontend_output");
  field(conf.input, "input");
}

HydraRosPipeline::HydraRosPipeline(const ros::NodeHandle& nh, int robot_id)
    : HydraPipeline(config::fromRos<PipelineConfig>(nh), robot_id),
      config_(config::checkValid(config::fromRos<HydraRosConfig>(nh))),
      nh_(nh) {}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  const auto& pipeline_config = GlobalInfo::instance().getConfig();
  initFrontend();
  initBackend();
  initReconstruction();
  if (pipeline_config.enable_lcd) {
    initLCD();
    bow_sub_.reset(new BowSubscriber(nh_, shared_state_));
  }

  const auto reconstruction = getModule<ReconstructionModule>("reconstruction");
  CHECK(reconstruction);
  input_module_.reset(new RosInputModule(config_.input, reconstruction->queue()));
}

void HydraRosPipeline::initFrontend() {
  ros::NodeHandle fnh(nh_, "frontend");
  const auto logs = GlobalInfo::instance().getLogs();
  FrontendModule::Ptr frontend =
      config::createFromROS<FrontendModule>(fnh, frontend_dsg_, shared_state_, logs);
  if (config_.enable_frontend_output) {
    CHECK(frontend) << "Frontend module required!";
    frontend->addSink(std::make_shared<RosFrontendPublisher>(fnh));
  }

  modules_["frontend"] = frontend;
}

void HydraRosPipeline::initBackend() {
  ros::NodeHandle bnh(nh_, "backend");
  const auto logs = GlobalInfo::instance().getLogs();
  BackendModule::Ptr backend = config::createFromROS<BackendModule>(
      bnh, backend_dsg_, shared_state_, GlobalInfo::instance().getLogs());
  CHECK(backend) << "Failed to construct backend!";
  backend->addSink(std::make_shared<RosBackendPublisher>(bnh));
  modules_["backend"] = backend;

  const auto frontend = getModule<FrontendModule>("frontend");
  if (!frontend) {
    LOG(ERROR) << "Invalid frontend module! Not setting 2D places update";
    return;
  }

  if (frontend->config.surface_places) {
    auto places_functor =
        std::make_shared<Update2dPlacesFunctor>(backend->config.places2d_config);
    backend->setUpdateFunctor(DsgLayers::MESH_PLACES, places_functor);
  }

  if (frontend->config.use_frontiers && frontend->config.frontier_places) {
    auto frontiers_functor =
        std::make_shared<UpdateFrontiersFunctor>(backend->config.frontier_config);
    backend->setUpdateFunctor(DsgLayers::BUILDINGS + 1, frontiers_functor);
  }
}

void HydraRosPipeline::initReconstruction() {
  const auto frontend = getModule<FrontendModule>("frontend");
  if (!frontend) {
    LOG(ERROR) << "Invalid frontend module: disabling reconstruction";
    return;
  }

  ros::NodeHandle rnh(nh_, "reconstruction");
  modules_["reconstruction"] =
      config::createFromROS<ReconstructionModule>(rnh, frontend->getQueue());
}

void HydraRosPipeline::initLCD() {
  auto lcd_config = config::fromRos<LoopClosureConfig>(nh_);
  lcd_config.detector.num_semantic_classes = GlobalInfo::instance().getTotalLabels();
  VLOG(1) << "Number of classes for LCD: " << lcd_config.detector.num_semantic_classes;
  config::checkValid(lcd_config);

  shared_state_->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
  auto lcd = std::make_shared<LoopClosureModule>(lcd_config, shared_state_);
  modules_["lcd"] = lcd;

  if (lcd_config.detector.enable_agent_registration) {
    lcd->getDetector().setRegistrationSolver(0,
                                             std::make_unique<lcd::DsgAgentSolver>());
  }
}

}  // namespace hydra
