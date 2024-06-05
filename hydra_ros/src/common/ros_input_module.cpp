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
#include "hydra_ros/common/ros_input_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/hydra_config.h>

#include "hydra_ros/utils/lookup_tf.h"

namespace hydra {

void declare_config(RosInputModule::Config& conf) {
  using namespace config;
  name("RosInputModule::Config");
  base<InputModule::Config>(conf);
  field(conf.ns, "ns");
  field(conf.tf_wait_duration_s, "tf_wait_duration_s");
  field(conf.tf_buffer_size_s, "tf_buffer_size_s");
}

RosInputModule::RosInputModule(const Config& config, const OutputQueue::Ptr& queue)
    : InputModule(config, queue), nh_(ros::NodeHandle(config.ns)) {
  buffer_.reset(new tf2_ros::Buffer(ros::Duration(config.tf_buffer_size_s)));
  tf_listener_.reset(new tf2_ros::TransformListener(*buffer_));

  receiver_ = config.receiver.create();
}

RosInputModule::~RosInputModule() = default;

std::string RosInputModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

PoseStatus RosInputModule::getBodyPose(uint64_t timestamp_ns) {
  ros::Time curr_ros_time;
  curr_ros_time.fromNSec(timestamp_ns);
  const auto pose_status = lookupTransform(*buffer_,
                                           curr_ros_time,
                                           HydraConfig::instance().getFrames().odom,
                                           HydraConfig::instance().getFrames().robot,
                                           5,  // max tries
                                           config.tf_wait_duration_s);
  return pose_status;
}

}  // namespace hydra
