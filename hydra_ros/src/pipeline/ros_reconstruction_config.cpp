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
#include "hydra_ros/pipeline/ros_reconstruction_config.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <geometry_msgs/TransformStamped.h>
#include <hydra/reconstruction/reconstruction_module.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

namespace hydra {

void declare_config(RosReconstructionConfig& conf) {
  using namespace config;
  name("RosReconstructionConfig");
  base<ReconstructionConfig>(conf);
  enum_field(conf.extrinsics_mode,
             "extrinsics_mode",
             {{ExtrinsicsLookupMode::USE_KIMERA, "USE_KIMERA"},
              {ExtrinsicsLookupMode::USE_TF, "USE_TF"},
              {ExtrinsicsLookupMode::USE_LOADED_PARAMS, "USE_LOADED_PARAMS"}});

  switch (conf.extrinsics_mode) {
    case ExtrinsicsLookupMode::USE_KIMERA:
      field(conf.kimera_extrinsics_file, "kimera_extrinsics_file");
      break;
    case ExtrinsicsLookupMode::USE_TF:
      field(conf.sensor_frame, "sensor_frame");
      break;
    case ExtrinsicsLookupMode::USE_LOADED_PARAMS:
    default:
      break;
  }

  field(conf.use_image_receiver, "use_image_receiver");
  field(conf.publish_pointcloud, "publish_pointcloud");
  field(conf.enable_output_queue, "enable_reconstruction_output_queue");
  field(conf.pointcloud_separation_s, "pointcloud_separation_s");
  field(conf.tf_wait_duration_s, "tf_wait_duration_s");
  field(conf.tf_buffer_size_s, "tf_buffer_size_s");
  field(conf.image_queue_size, "image_queue_size");
}

// this is technically not threadsafe w.r.t. to spinning in another thread.
// but this gets called in the constructor, so should be fine for now
bool lookupExtrinsicsFromTf(RosReconstructionConfig& config) {
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  ros::WallRate tf_wait_rate(1.0 / config.tf_wait_duration_s);

  bool have_transform = false;
  std::string err_str;
  LOG(INFO) << "Looking up extrinsics via TF: " << config.robot_frame << " -> "
            << config.sensor_frame;
  while (ros::ok()) {
    if (buffer.canTransform(config.robot_frame,
                            config.sensor_frame,
                            ros::Time(),
                            ros::Duration(0),
                            &err_str)) {
      have_transform = true;
      break;
    }

    tf_wait_rate.sleep();
    ros::spinOnce();
  }

  if (!have_transform) {
    LOG(ERROR) << "Failed to look up: " << config.robot_frame << " to "
               << config.sensor_frame;
    return false;
  }

  geometry_msgs::TransformStamped transform;
  try {
    transform =
        buffer.lookupTransform(config.robot_frame, config.sensor_frame, ros::Time());
  } catch (const tf2::TransformException& ex) {
    LOG(ERROR) << "Failed to look up: " << config.robot_frame << " to "
               << config.sensor_frame;
    return false;
  }

  geometry_msgs::Pose curr_pose;
  curr_pose.position.x = transform.transform.translation.x;
  curr_pose.position.y = transform.transform.translation.y;
  curr_pose.position.z = transform.transform.translation.z;
  curr_pose.orientation = transform.transform.rotation;
  tf2::convert(curr_pose.position, config.body_t_camera);
  tf2::convert(curr_pose.orientation, config.body_R_camera);
  config.body_R_camera.normalize();
  return true;
}

bool loadReconstructionExtrinsics(RosReconstructionConfig& config) {
  bool extrinsics_valid = true;
  switch (config.extrinsics_mode) {
    case ExtrinsicsLookupMode::USE_KIMERA:
      extrinsics_valid =
          loadExtrinsicsFromKimera(config, config.kimera_extrinsics_file);
      break;
    case ExtrinsicsLookupMode::USE_TF:
      extrinsics_valid = lookupExtrinsicsFromTf(config);
      break;
    case ExtrinsicsLookupMode::USE_LOADED_PARAMS:
    default:
      break;
  }

  return extrinsics_valid;
}

}  // namespace hydra
