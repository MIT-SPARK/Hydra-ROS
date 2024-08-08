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
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/global_info.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>

#include "hydra_ros/utils/pose_cache.h"
#include "hydra_ros/utils/tf_lookup.h"

namespace hydra {

struct CameraInfoFunctor {
  void callback(const sensor_msgs::CameraInfo::ConstPtr& info) { msg = info; }

  sensor_msgs::CameraInfo::ConstPtr msg;
};

struct TempCameraConfig : Camera::Config {};

void declare_config(TempCameraConfig& config) {
  using namespace config;
  name("Sensor::Config");
  base<Sensor::Config>(config);
}

void fillConfigFromInfo(const sensor_msgs::CameraInfo& msg,
                        Camera::Config& cam_config) {
  cam_config.width = msg.width;
  cam_config.height = msg.height;
  cam_config.fx = msg.K[0];
  cam_config.fy = msg.K[4];
  cam_config.cx = msg.K[2];
  cam_config.cy = msg.K[5];
}

std::optional<sensor_msgs::CameraInfo> getCameraInfo(const std::string& ns) {
  ros::NodeHandle nh(ns);
  CameraInfoFunctor functor;
  ros::Subscriber sub =
      nh.subscribe("camera_info", 1, &CameraInfoFunctor::callback, &functor);

  const auto resolved_topic = nh.resolveName("camera_info");
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
    return std::nullopt;
  }

  return *functor.msg;
}

std::optional<sensor_msgs::CameraInfo> getCameraInfo(const rosbag::Bag& bag,
                                                     const std::string& topic) {
  rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{topic}));
  for (const auto& m : view) {
    const auto msg = m.instantiate<sensor_msgs::CameraInfo>();
    if (!msg) {
      LOG(ERROR) << "Topic '" << topic << "' is invalid!";
      return std::nullopt;
    }

    return *msg;
  }

  LOG(ERROR) << "Failed to find topic '" << topic << "' in bag!'";
  return std::nullopt;
}

ParamSensorExtrinsics::Config lookupExtrinsics(const std::string& sensor_frame) {
  const auto robot_frame = GlobalInfo::instance().getFrames().robot;
  const auto pose = lookupTransform(robot_frame, sensor_frame);
  CHECK(pose.is_valid) << "Could not look up extrinsics from ros!";

  ParamSensorExtrinsics::Config config;
  config.body_R_sensor = pose.target_R_source;
  config.body_p_sensor = pose.target_p_source;
  return config;
}

ParamSensorExtrinsics::Config lookupExtrinsics(const rosbag::Bag& bag,
                                               const std::string& sensor_frame) {
  PoseCache cache(bag, true);
  const auto robot_frame = GlobalInfo::instance().getFrames().robot;
  const auto pose = cache.lookupPose(0, robot_frame, sensor_frame);
  CHECK(pose) << "Could not look up extrinsics from bag!";

  ParamSensorExtrinsics::Config config;
  config.body_R_sensor = pose.to_R_from;
  config.body_p_sensor = pose.to_p_from;
  return config;
}

RosExtrinsics::RosExtrinsics(const Config&) {
  throw std::runtime_error("Cannot directly insantiate RosExtrinsics object!");
}

RosCamera::RosCamera(const Config&) {
  throw std::runtime_error("Cannot directly insantiate RosExtrinsics object!");
}

RosbagCamera::RosbagCamera(const Config&) {
  throw std::runtime_error("Cannot directly insantiate RosExtrinsics object!");
}

void declare_config(RosExtrinsics::Config& config) {
  using namespace config;
  name("RosExtrinsics::Config");
  field(config.sensor_frame, "sensor_frame");
}

void declare_config(RosCamera::Config& config) {
  using namespace config;
  name("RosCamera::Config");
  base<Sensor::Config>(config);
  field(config.ns, "ns");
}

void declare_config(RosbagCamera::Config& config) {
  using namespace config;
  name("RosbagCamera::Config");
  base<Sensor::Config>(config);
  field(config.topic, "camera_info_topic");
  checkCondition(!config.topic.empty(), "camera info topic required");
}

namespace input {

using VirtualSensor = config::VirtualConfig<Sensor>;

VirtualSensor loadExtrinsics(const VirtualSensor& sensor,
                             const std::string& sensor_frame = "",
                             const rosbag::Bag* bag = nullptr) {
  if (!sensor) {
    return sensor;
  }

  auto base_contents = config::toYaml(sensor);
  const auto base_config = config::fromYaml<Sensor::Config>(base_contents);
  if (!base_config.extrinsics || base_config.extrinsics.getType() != "ros") {
    return sensor;
  }

  const auto contents = config::toYaml(base_config.extrinsics);
  const auto derived = config::fromYaml<RosExtrinsics::Config>(contents);
  const auto frame = derived.sensor_frame.empty() ? sensor_frame : derived.sensor_frame;
  if (frame.empty()) {
    LOG(ERROR) << "sensor frame required if not constructing from camera info!";
    return {};
  }

  const auto info = bag ? lookupExtrinsics(*bag, frame) : lookupExtrinsics(frame);

  config::VirtualConfig<SensorExtrinsics> new_config(info);
  base_contents["extrinsics"] = config::toYaml(new_config);
  return config::fromYaml<VirtualSensor>(base_contents);
}

VirtualSensor loadSensor(const VirtualSensor& sensor, const std::string& sensor_name) {
  if (!sensor || sensor.getType() != "camera_info") {
    return loadExtrinsics(sensor);
  }

  const auto contents = config::toYaml(sensor);
  const auto derived = config::fromYaml<RosCamera::Config>(contents);

  const auto ns =
      derived.ns.empty() ? "~/input/" + sensor_name + std::string("/rgb") : derived.ns;
  const auto msg = getCameraInfo(ns);
  if (!msg) {
    return {};
  }

  // get base class fields (all other derived fields will be overriden by ros)
  Camera::Config config = config::fromYaml<TempCameraConfig>(contents);
  fillConfigFromInfo(*msg, config);

  VirtualSensor new_sensor(config);
  new_sensor = loadExtrinsics(new_sensor, msg->header.frame_id);
  LOG(INFO) << "Initialized camera as " << std::endl << config::toString(new_sensor);
  return new_sensor;
}

VirtualSensor loadSensor(const rosbag::Bag& bag, const VirtualSensor& sensor) {
  if (!sensor || sensor.getType() != "rosbag_camera_info") {
    return loadExtrinsics(sensor, "", &bag);
  }

  const auto contents = config::toYaml(sensor);
  const auto derived = config::fromYaml<RosbagCamera::Config>(contents);
  const auto msg = getCameraInfo(bag, derived.topic);
  if (!msg) {
    return {};
  }

  // get base class fields (all other derived fields will be overriden by ros)
  Camera::Config config = config::fromYaml<TempCameraConfig>(contents);
  fillConfigFromInfo(*msg, config);

  VirtualSensor new_sensor(config);
  new_sensor = loadExtrinsics(new_sensor, msg->header.frame_id, &bag);
  LOG(INFO) << "Initialized camera as " << std::endl << config::toString(new_sensor);
  return new_sensor;
}

}  // namespace input

}  // namespace hydra
