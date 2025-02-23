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
#include <ianvs/node_handle.h>

#include <std_msgs/msg/string.hpp>
// TODO(nathan) don't use internal tools
#include <config_utilities/internal/yaml_utils.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>

#include <functional>
#include <memory>

namespace hydra::visualizer {

// TODO(nathan) make threadsafe
// TODO(nathan) save
// TODO(nathan) logging
// TODO(nathan) consider show
template <typename Config>
class ConfigWrapper {
 public:
  using Ptr = std::shared_ptr<ConfigWrapper<Config>>;
  using UpdateCallback = std::function<void(const Config&)>;

  ConfigWrapper(ianvs::NodeHandle _nh,
                const std::string& ns,
                const Config& initial = {})
      : changed_(true),
        ns_(ns),
        config_(initial),
        curr_contents_(config::toYaml(config_)) {
    auto nh = _nh / ns;
    sub_ = nh.create_subscription<std_msgs::msg::String>(
        "set", 1, &ConfigWrapper::update, this);
  }

  ConfigWrapper(const ConfigWrapper<Config>& other) = delete;
  ConfigWrapper(ConfigWrapper<Config>&& other) = delete;
  ConfigWrapper<Config>& operator=(const ConfigWrapper<Config>& other) = delete;
  ConfigWrapper<Config>& operator=(ConfigWrapper<Config>&& other) = delete;

  bool hasChange() const { return changed_; }

  void clearChangeFlag() { changed_ = false; }

  const Config& get() const { return config_; };

  void setUpdateCallback(const UpdateCallback& callback) {
    on_update_callback_ = callback;
  }

 private:
  void update(const std_msgs::msg::String::ConstSharedPtr& msg) {
    YAML::Node update;
    try {
      update = YAML::Load(msg->data);
    } catch (const std::exception& e) {
      // LOG(ERROR) << "Failed to parse given update '" << msg->data << "'";
      return;
    }

    config::internal::mergeYamlNodes(curr_contents_, update);
    config::updateFromYaml(config_, curr_contents_);
    changed_ = true;
    if (on_update_callback_) {
      on_update_callback_(config_);
    }
  }

 private:
  bool changed_;
  std::string ns_;
  Config config_;

  YAML::Node curr_contents_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  UpdateCallback on_update_callback_;
};

}  // namespace hydra::visualizer
