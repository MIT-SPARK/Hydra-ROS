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
#include <hydra/backend/backend_module.h>
#include <ianvs/node_handle.h>
#include <spark_dsg/metadata.h>

#include <mutex>

#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>

namespace hydra {

/**
 * @brief Class that listens to exernal updates to the meta data of the scene graph,
 * which are received as JSON strings via ROS topics.
 */
class RosMetaDataListener : public BackendModule::Sink {
 public:
  struct Config {
    //! @brief Input topic to listen to for meta data
    std::string topic_name = "~/meta_data_in";
  } const config;

  explicit RosMetaDataListener(const Config& config);

  virtual ~RosMetaDataListener() = default;

  void call(uint64_t,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph&) const override;

  std::string printInfo() const override;

 protected:
  void callback(const std_msgs::msg::String::ConstSharedPtr& msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  mutable spark_dsg::Metadata meta_data_;
  mutable std::mutex mutex_;
};

}  // namespace hydra
