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
#include "hydra_ros/reconstruction/ros_sensors.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/common/hydra_config.h>
#include <sensor_msgs/CameraInfo.h>

#include "hydra_ros/utils/lookup_tf.h"

namespace hydra {

using config::internal::ConfigFactory;
using config::internal::ModuleMapBase;
using config::internal::typeInfo;

RosSensorExtrinsics::RosSensorExtrinsics(const RosSensorExtrinsics::Config& config)
    : SensorExtrinsics() {
  config::checkValid(config);
  const auto pose_status =
      lookupTransform(HydraConfig::instance().getFrames().robot, config.sensor_frame);
  CHECK(pose_status.is_valid) << "Could not look up extrinsics from ros!";
  body_R_sensor = pose_status.target_R_source;
  body_p_sensor = pose_status.target_p_source;
  VLOG(5) << "body_R_sensor: {w: " << body_R_sensor.w() << ", x: " << body_R_sensor.x()
          << ", y: " << body_R_sensor.y() << ", z: " << body_R_sensor.z() << "}";
  VLOG(5) << "body_p_sensor: [" << body_p_sensor.x() << ", " << body_p_sensor.y()
          << ", " << body_p_sensor.z() << "]";
}

RosIntrinsicsRegistration::RosIntrinsicsRegistration(const std::string& name) {
  ConfigFactory<Sensor>::addEntry<RosCameraIntrinsics::Config>(name);
  ModuleMapBase<std::function<Sensor*(const YAML::Node&)>>::addEntry(
      name,
      [](const YAML::Node& data) {
        RosCameraIntrinsics::Config config;
        config::internal::Visitor::setValues(config, data);
        return new Camera(RosCameraIntrinsics::makeCameraConfig(data, config));
      },
      typeInfo<RosCameraIntrinsics>());
}

struct CameraInfoFunctor {
  void callback(const sensor_msgs::CameraInfo::ConstPtr& info) { msg = info; }

  sensor_msgs::CameraInfo::ConstPtr msg;
};

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
  cam_config.width = functor.msg->width;
  cam_config.height = functor.msg->height;
  cam_config.fx = functor.msg->K[0];
  cam_config.fy = functor.msg->K[4];
  cam_config.cx = functor.msg->K[2];
  cam_config.cy = functor.msg->K[5];
  LOG(INFO) << "Initialized Camera Info as " << std::endl
            << config::toString(cam_config);
  return cam_config;
}

void declare_config(RosSensorExtrinsics::Config& conf) {
  using namespace config;
  name("RosSensorExtrinsics::Config");
  field(conf.sensor_frame, "sensor_frame");
  checkCondition(!conf.sensor_frame.empty(), "sensor frame required");
}

void declare_config(RosCameraIntrinsics::Config& conf) {
  using namespace config;
  name("RosCameraIntrinsics::Config");
  base<Sensor::Config>(conf);
  field(conf.topic, "camera_info_topic");
  checkCondition(!conf.topic.empty(), "camera info topic required");
}

}  // namespace hydra
