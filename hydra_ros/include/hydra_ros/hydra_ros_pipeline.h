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
#pragma once
#include <hydra/active_window/active_window_module.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/hydra_pipeline.h>
#include <hydra/frontend/graph_builder.h>
#include <ros/ros.h>

#include "hydra_ros/input/feature_receiver.h"
#include "hydra_ros/input/ros_input_module.h"

namespace hydra {

class BowSubscriber;

class HydraRosPipeline : public HydraPipeline {
 public:
  struct Config {
    config::VirtualConfig<ActiveWindowModule> active_window{
        ReconstructionModule::Config()};
    config::VirtualConfig<GraphBuilder> frontend{GraphBuilder::Config()};
    config::VirtualConfig<BackendModule> backend{BackendModule::Config()};
    bool enable_frontend_output = true;
    RosInputModule::Config input;
    config::VirtualConfig<FeatureReceiver> features;
  } const config;

  HydraRosPipeline(const ros::NodeHandle& nh, int robot_id);

  virtual ~HydraRosPipeline();

  void init() override;

  void stop() override;

 protected:
  virtual void initLCD();

 protected:
  ros::NodeHandle nh_;
  std::shared_ptr<ActiveWindowModule> active_window_;
  std::shared_ptr<GraphBuilder> frontend_;
  std::shared_ptr<BackendModule> backend_;

  std::unique_ptr<BowSubscriber> bow_sub_;
};

void declare_config(HydraRosPipeline::Config& config);

}  // namespace hydra
