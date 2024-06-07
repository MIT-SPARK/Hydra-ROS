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
#include "hydra_ros/input/ros_sensors.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>

#include "hydra_ros/utils/lookup_tf.h"
#include "hydra_ros/utils/pose_cache.h"

namespace hydra {

using config::internal::ConfigFactory;
using config::internal::ModuleMapBase;
using config::internal::typeInfo;

struct CameraInfoFunctor {
  void callback(const sensor_msgs::CameraInfo::ConstPtr& info) { msg = info; }

  sensor_msgs::CameraInfo::ConstPtr msg;
};

void fillConfigFromInfo(const sensor_msgs::CameraInfo& msg,
                        Camera::Config& cam_config) {
  cam_config.width = msg.width;
  cam_config.height = msg.height;
  cam_config.fx = msg.K[0];
  cam_config.fy = msg.K[4];
  cam_config.cx = msg.K[2];
  cam_config.cy = msg.K[5];
}

RosSensorExtrinsics::RosSensorExtrinsics(const RosSensorExtrinsics::Config& config)
    : SensorExtrinsics() {
  config::checkValid(config);
  const auto pose_status =
      lookupTransform(GlobalInfo::instance().getFrames().robot, config.sensor_frame);
  CHECK(pose_status.is_valid) << "Could not look up extrinsics from ros!";
  body_R_sensor = pose_status.target_R_source;
  body_p_sensor = pose_status.target_p_source;
  VLOG(5) << "body_R_sensor: {w: " << body_R_sensor.w() << ", x: " << body_R_sensor.x()
          << ", y: " << body_R_sensor.y() << ", z: " << body_R_sensor.z() << "}";
  VLOG(5) << "body_p_sensor: [" << body_p_sensor.x() << ", " << body_p_sensor.y()
          << ", " << body_p_sensor.z() << "]";
}

RosbagExtrinsics::RosbagExtrinsics(const RosbagExtrinsics::Config& config)
    : SensorExtrinsics() {
  config::checkValid(config);
  PoseCache::Config cache_config;
  cache_config.bag_path = config.bag_path;
  cache_config.static_only = true;
  PoseCache cache(cache_config);

  const auto pose_status = cache.lookupPose(
      0, GlobalInfo::instance().getFrames().robot, config.sensor_frame);
  CHECK(pose_status) << "Could not look up extrinsics from bag!";
  body_R_sensor = pose_status.to_R_from;
  body_p_sensor = pose_status.to_p_from;
  VLOG(5) << "body_R_sensor: {w: " << body_R_sensor.w() << ", x: " << body_R_sensor.x()
          << ", y: " << body_R_sensor.y() << ", z: " << body_R_sensor.z() << "}";
  VLOG(5) << "body_p_sensor: [" << body_p_sensor.x() << ", " << body_p_sensor.y()
          << ", " << body_p_sensor.z() << "]";
}

RosIntrinsicsRegistration::RosIntrinsicsRegistration(const std::string& name) {
  ConfigFactory<Sensor>::addEntry<RosCameraIntrinsics::Config>(name);
  ModuleMapBase<std::function<Sensor*(const YAML::Node&)>>::addEntry(
      name,
      [name](const YAML::Node& data) -> Camera* {
        if (name == "camera_info") {
          RosCameraIntrinsics::Config config;
          config::internal::Visitor::setValues(config, data);
          return new Camera(RosCameraIntrinsics::makeCameraConfig(data, config));
        } else if (name == "rosbag_camera_info") {
          RosbagCameraIntrinsics::Config config;
          config::internal::Visitor::setValues(config, data);
          return new Camera(RosbagCameraIntrinsics::makeCameraConfig(data, config));
        } else {
          return nullptr;
        }
      },
      typeInfo<RosCameraIntrinsics>());
}

Camera::Config RosCameraIntrinsics::makeCameraConfig(const YAML::Node& data,
                                                     const Config& config) {
  sensor_msgs::CameraInfo::ConstPtr msg;

  ros::NodeHandle nh("~");
  const auto resolved_topic = nh.resolveName(config.topic);
  CameraInfoFunctor functor;
  ros::Subscriber sub =
      nh.subscribe(config.topic, 1, &CameraInfoFunctor::callback, &functor);

  LOG(INFO) << "Waiting for CameraInfo on " << resolved_topic
            << " to initialize sensor model";
  ros::WallRate r(10);
  while (ros::ok()) {
    if (functor.msg) {
      break;
    }

    ros::spinOnce();
    r.sleep();
  }

  if (!functor.msg) {
    LOG(ERROR) << "did not receive message on " << resolved_topic;
    return {};
  }

  Camera::Config cam_config;
  config::internal::Visitor::setValues(static_cast<Sensor::Config&>(cam_config), data);
  fillConfigFromInfo(*functor.msg, cam_config);
  LOG(INFO) << "Initialized camera as " << std::endl << config::toString(cam_config);
  return cam_config;
}

Camera::Config RosbagCameraIntrinsics::makeCameraConfig(const YAML::Node& data,
                                                        const Config& config) {
  LOG(INFO) << "Loading camera intrinsics from " << config.bag_path;

  rosbag::Bag bag;
  bag.open(config.bag_path, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{config.topic}));

  Camera::Config cam_config;
  for (const auto& m : view) {
    const auto msg = m.instantiate<sensor_msgs::CameraInfo>();
    if (!msg) {
      LOG(ERROR) << "Topic '" << config.topic << "' is invalid!";
      break;
    }

    config::internal::Visitor::setValues(static_cast<Sensor::Config&>(cam_config),
                                         data);
    cam_config.width = msg->width;
    cam_config.height = msg->height;
    cam_config.cx = msg->K[2];
    cam_config.cy = msg->K[5];
    cam_config.fx = msg->K[0];
    cam_config.fy = msg->K[4];
    LOG(INFO) << "Initialized Camera Info as " << std::endl
              << config::toString(cam_config);
    return cam_config;
  }

  bag.close();

  LOG(ERROR) << "Failed to find topic '" << config.topic << "' in bag!'";
  return cam_config;
}

void declare_config(RosSensorExtrinsics::Config& conf) {
  using namespace config;
  name("RosSensorExtrinsics::Config");
  field(conf.sensor_frame, "sensor_frame");
  checkCondition(!conf.sensor_frame.empty(), "sensor frame required");
}

void declare_config(RosbagExtrinsics::Config& config) {
  using namespace config;
  name("RosbagExtrinsics::Config");
  field(config.sensor_frame, "sensor_frame");
  field<Path>(config.bag_path, "bag_path");
  checkCondition(!config.sensor_frame.empty(), "sensor frame required");
  check<Path::Exists>(config.bag_path, "bag_path");
}

void declare_config(RosCameraIntrinsics::Config& conf) {
  using namespace config;
  name("RosCameraIntrinsics::Config");
  base<Sensor::Config>(conf);
  field(conf.topic, "camera_info_topic");
  checkCondition(!conf.topic.empty(), "camera info topic required");
}

void declare_config(RosbagCameraIntrinsics::Config& config) {
  using namespace config;
  name("RosbagCameraIntrinsics::Config");
  base<Sensor::Config>(config);
  field(config.topic, "camera_info_topic");
  field<Path>(config.bag_path, "bag_path");
  checkCondition(!config.topic.empty(), "camera info topic required");
  check<Path::Exists>(config.bag_path, "bag_path");
}

}  // namespace hydra
