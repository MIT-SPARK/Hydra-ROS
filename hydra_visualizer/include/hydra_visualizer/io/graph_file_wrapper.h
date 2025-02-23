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
#include <config_utilities/factory.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <filesystem>

#include "hydra_visualizer/io/graph_wrapper.h"

namespace hydra {

class GraphFileWrapper : public GraphWrapper {
 public:
  struct Config {
    std::filesystem::path filepath;
    std::string wrapper_ns = "~/graph";
  } const config;

  explicit GraphFileWrapper(const Config& config);

  bool hasChange() const override;

  void clearChangeFlag() override;

  StampedGraph get() const override;

  bool reload(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void load(const std_msgs::String& msg);

 private:
  ros::NodeHandle nh_;
  bool has_change_;
  std::filesystem::path filepath_;
  spark_dsg::DynamicSceneGraph::Ptr graph_;
  ros::ServiceServer service_;
  ros::Subscriber sub_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphWrapper,
                                     GraphFileWrapper,
                                     GraphFileWrapper::Config>("GraphFromFile");
};

void declare_config(GraphFileWrapper::Config& config);

}  // namespace hydra
