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
#include "hydra_ros/utils/lookup_tf.h"

#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

namespace hydra {

PoseStatus lookupTransform(const std::string& target,
                           const std::string& source,
                           double wait_duration_s,
                           int verbosity) {
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  return lookupTransform(
      buffer, std::nullopt, target, source, std::nullopt, wait_duration_s, verbosity);
}

PoseStatus lookupTransform(const tf2_ros::Buffer& buffer,
                           const std::optional<ros::Time>& stamp,
                           const std::string& target,
                           const std::string& source,
                           std::optional<size_t> max_tries,
                           double wait_duration_s,
                           int verbosity) {
  ros::WallRate tf_wait_rate(1.0 / wait_duration_s);
  std::string stamp_suffix;
  if (stamp) {
    std::stringstream ss;
    ss << " @ " << stamp.value().toNSec() << " [ns]";
    stamp_suffix = ss.str();
  }

  bool have_transform = false;
  std::string err_str;
  VLOG(verbosity) << "Looking up transform " << target << "_T_" << source
                  << stamp_suffix;

  const auto lookup_time = stamp.value_or(ros::Time());
  size_t attempt_number = 0;
  while (ros::ok()) {
    VLOG(verbosity) << "Attempting to lookup tf @ " << lookup_time.toNSec()
                    << " [ns]: " << attempt_number << " / "
                    << (max_tries ? std::to_string(max_tries.value()) : "n/a");
    if (max_tries && attempt_number >= *max_tries) {
      break;
    }

    if (buffer.canTransform(target, source, lookup_time, ros::Duration(0), &err_str)) {
      have_transform = true;
      break;
    }

    ++attempt_number;
    tf_wait_rate.sleep();
    ros::spinOnce();
  }

  if (!have_transform) {
    LOG(ERROR) << "Failed to find: " << target << "_T_" << source << stamp_suffix
               << ": " << err_str;
    return {false, {}, {}};
  }

  geometry_msgs::TransformStamped transform;
  try {
    transform = buffer.lookupTransform(target, source, lookup_time);
  } catch (const tf2::TransformException& ex) {
    LOG(ERROR) << "Failed to look up: " << target << "_T_" << source << stamp_suffix;
    return {false, {}, {}};
  }

  geometry_msgs::Pose curr_pose;
  curr_pose.position.x = transform.transform.translation.x;
  curr_pose.position.y = transform.transform.translation.y;
  curr_pose.position.z = transform.transform.translation.z;
  curr_pose.orientation = transform.transform.rotation;

  PoseStatus to_return;
  to_return.is_valid = true;
  tf2::convert(curr_pose.position, to_return.target_p_source);
  tf2::convert(curr_pose.orientation, to_return.target_R_source);
  to_return.target_R_source.normalize();
  return to_return;
}

}  // namespace hydra
