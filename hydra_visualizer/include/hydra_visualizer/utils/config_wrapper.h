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
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace hydra::visualizer {

// TODO(nathan) make threadsafe
template <typename Config>
class ConfigWrapper {
 public:
  using Ptr = std::shared_ptr<ConfigWrapper<Config>>;
  using Server = dynamic_reconfigure::Server<Config>;
  using UpdateCallback = std::function<void(const Config&)>;

  ConfigWrapper(const ros::NodeHandle& nh, const std::string& ns)
      : nh_(nh, ns), changed_(true) {
    server_ = std::make_unique<Server>(nh_);
    server_->setCallback(boost::bind(&ConfigWrapper<Config>::update, this, _1, _2));
  }

  bool hasChange() const { return changed_; }

  void clearChangeFlag() { changed_ = false; }

  const Config& get() const { return config_; };

  void setUpdateCallback(const UpdateCallback& callback) {
    on_update_callback_ = callback;
  }

 private:
  void update(Config& config, uint32_t) {
    config_ = config;
    changed_ = true;
    if (on_update_callback_) {
      on_update_callback_(config_);
    }
  }

 private:
  ros::NodeHandle nh_;

  bool changed_;
  Config config_;

  std::unique_ptr<Server> server_;
  UpdateCallback on_update_callback_;
};

}  // namespace hydra::visualizer
