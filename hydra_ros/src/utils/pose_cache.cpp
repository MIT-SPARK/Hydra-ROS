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
#include "hydra_ros/utils/pose_cache.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <geometry_msgs/Pose.h>
#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_msgs/TFMessage.h>

namespace hydra {

void fillBuffer(const rosbag::Bag& bag,
                bool static_only,
                std::shared_ptr<tf2::BufferCore>& buffer) {
  std::vector<std::string> topics{"/tf_static"};
  if (!static_only) {
    topics.push_back("/tf");
  }

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  const auto bag_duration = view.getEndTime() - view.getBeginTime();
  buffer = std::make_shared<tf2::BufferCore>(bag_duration + ros::Duration(10.0));

  for (const auto& m : view) {
    const auto msg = m.instantiate<tf2_msgs::TFMessage>();
    if (!msg) {
      LOG(ERROR) << "Found invalid message on '" << m.getTopic() << "'";
      continue;
    }

    const bool is_static = m.getTopic() == "/tf_static";
    for (const auto& tf : msg->transforms) {
      buffer->setTransform(tf, "rosbag", is_static);
    }
  }
}

PoseCache::PoseCache(const PoseCache::Config& config) {
  config::checkValid(config);
  LOG(INFO) << "Loading poses from " << config.bag_path;


  rosbag::Bag bag;
  bag.open(config.bag_path, rosbag::bagmode::Read);
  fillBuffer(bag, config.static_only, buffer_);
  bag.close();
}

PoseCache::PoseCache(const rosbag::Bag& bag, bool static_only) {
  fillBuffer(bag, static_only, buffer_);
}

PoseCache::PoseResult PoseCache::lookupPose(uint64_t timestamp_ns,
                                            const std::string& to_frame,
                                            const std::string& from_frame) const {
  PoseResult result;
  try {
    ros::Time stamp;
    stamp.fromNSec(timestamp_ns);
    auto msg = buffer_->lookupTransform(to_frame, from_frame, stamp);

    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = msg.transform.translation.x;
    curr_pose.position.y = msg.transform.translation.y;
    curr_pose.position.z = msg.transform.translation.z;
    curr_pose.orientation = msg.transform.rotation;

    result.valid = true;
    tf2::convert(curr_pose.position, result.to_p_from);
    tf2::convert(curr_pose.orientation, result.to_R_from);
    result.to_R_from.normalize();
  } catch (const tf2::TransformException& e) {
    LOG(ERROR) << "Unable to find pose @ " << timestamp_ns << " [ns] between '"
               << from_frame << "' and '" << to_frame << "': " << e.what();
  }

  return result;
}

void declare_config(PoseCache::Config& config) {
  using namespace config;
  name("RosCameraIntrinsics::Config");
  field<Path>(config.bag_path, "bag_path");
  field(config.static_only, "static_only");
  check<Path::Exists>(config.bag_path, "bag_path");
}

}  // namespace hydra
