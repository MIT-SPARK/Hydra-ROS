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
#include "hydra_ros/backend/ros_meta_data_listener.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/common_types.h>
#include <hydra/common/global_info.h>
#include <ianvs/node_handle.h>
#include <kimera_pgmo_ros/visualization_functions.h>

namespace hydra {

namespace {
static const auto registration =
    config::RegistrationWithConfig<BackendModule::Sink,
                                   RosMetaDataListener,
                                   RosMetaDataListener::Config>("RosMetaDataListener");
}  // namespace

void declare_config(RosMetaDataListener::Config& config) {
  using namespace config;
  name("RosMetaDataListener::Config");
  field(config.topic_name, "topic_name");
}

RosMetaDataListener::RosMetaDataListener(const Config& config) {
  sub_ = ianvs::NodeHandle::this_node().create_subscription<std_msgs::msg::String>(
      config.topic_name,
      rclcpp::QoS(100),
      std::bind(&RosMetaDataListener::callback, this, std::placeholders::_1));
}

void RosMetaDataListener::call(uint64_t,
                               const DynamicSceneGraph& graph,
                               const kimera_pgmo::DeformationGraph&) const {
  if (meta_data_.empty()) {
    return;
  }
  // TODO(lschmid): This isn't beutiful, revisit the callback interfaces if we want to
  // expose also setting things properly.
  std::lock_guard<std::mutex> lock(mutex_);
  const_cast<DynamicSceneGraph&>(graph).metadata.add(meta_data_);
  meta_data_.clear();
}

std::string RosMetaDataListener::printInfo() const { return config::toString(config); }

void RosMetaDataListener::callback(const std_msgs::msg::String::ConstSharedPtr& msg) {
  nlohmann::json input_json;
  try {
    input_json = nlohmann::json::parse(msg->data);
  } catch (const nlohmann::json::parse_error& e) {
    LOG(ERROR) << "Failed to read meta data. JSON parse error: " << e.what();
    return;
  }

  // Accumulate incoming meta data till the next call.
  std::lock_guard<std::mutex> lock(mutex_);
  meta_data_.add(input_json);
}

}  // namespace hydra
