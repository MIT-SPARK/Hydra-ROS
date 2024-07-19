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
#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <gtest/gtest.h>
#include <hydra/input/lidar.h>
#include <hydra_ros/input/ros_sensors.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <Eigen/Geometry>

namespace hydra {

using VirtualSensor = config::VirtualConfig<Sensor>;

namespace {

template <typename T>
inline std::string getExportedConfig(const T& config) {
  VirtualSensor sensor(config);
  std::stringstream ss;
  ss << config::toYaml(sensor);
  return ss.str();
}

}  // namespace

class RosSensors : public ::testing::Test {
 public:
  RosSensors() = default;
  virtual ~RosSensors() = default;
  void SetUp() override {
    info_pub =
        nh.advertise<sensor_msgs::CameraInfo>("/some/camera/camera_info", 1, true);
    sensor_msgs::CameraInfo msg;
    msg.header.frame_id = "lidar";
    msg.height = 1;
    msg.width = 2;
    msg.K = {3, 0, 4, 0, 5, 6, 0, 0, 0};
    info_pub.publish(msg);

    expected_intrinsics.min_range = 10;
    expected_intrinsics.max_range = 15;
    expected_intrinsics.fx = 3;
    expected_intrinsics.cx = 4;
    expected_intrinsics.fy = 5;
    expected_intrinsics.cy = 6;
    expected_intrinsics.height = 1;
    expected_intrinsics.width = 2;

    expected_extrinsics.body_R_sensor = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);
    expected_extrinsics.body_p_sensor << 1.0, 2.0, 3.0;
  }

  ParamSensorExtrinsics::Config expected_extrinsics;
  Camera::Config expected_intrinsics;

  ros::NodeHandle nh;
  ros::Publisher info_pub;
};

TEST_F(RosSensors, TestNonCamera) {
  Lidar::Config config;
  config.horizontal_resolution = 1;
  config.vertical_resolution = 2;
  config.horizontal_fov = 3;
  config.vertical_fov = 4;
  config.is_asymmetric = true;
  config.vertical_fov_top = 5;
  config.min_range = 6;
  config.max_range = 7;
  config.extrinsics = IdentitySensorExtrinsics::Config();

  {  // non-ros should always be the same
    VirtualSensor sensor(config);
    const auto result = input::loadSensor(sensor, 0);
    ASSERT_TRUE(result);
    EXPECT_EQ(result.getType(), "lidar");
    const auto expected_yaml = getExportedConfig(config);
    const auto result_yaml = getExportedConfig(result);
    EXPECT_EQ(expected_yaml, result_yaml);
  }

  RosExtrinsics::Config extrinsics_config;
  config.extrinsics = extrinsics_config;

  {  // ros without a frame should not be valid
    VirtualSensor sensor(config);
    const auto result = input::loadSensor(sensor, 0);
    EXPECT_FALSE(result);
  }

  extrinsics_config.sensor_frame = "lidar";
  config.extrinsics = extrinsics_config;

  {  // ros without a frame should not be valid
    VirtualSensor sensor(config);
    const auto result = input::loadSensor(sensor, 0);
    ASSERT_TRUE(result);
    EXPECT_EQ(result.getType(), "lidar");

    config.extrinsics = expected_extrinsics;
    const auto expected_yaml = getExportedConfig(config);
    const auto result_yaml = getExportedConfig(result);
    EXPECT_EQ(expected_yaml, result_yaml);
  }
}

TEST_F(RosSensors, Camera) {
  RosCamera::Config config;
  config.ns = "/some/camera";
  config.min_range = 10;
  config.max_range = 15;
  config.extrinsics = IdentitySensorExtrinsics::Config();

  {  // should be able to parse intrinsics and extrinsics separately
    VirtualSensor sensor(config);
    const auto result = input::loadSensor(sensor, 0);
    ASSERT_TRUE(result);
    EXPECT_EQ(result.getType(), "camera");

    Camera::Config expected = expected_intrinsics;
    expected.extrinsics = IdentitySensorExtrinsics::Config();
    const auto expected_yaml = getExportedConfig(expected);
    const auto result_yaml = getExportedConfig(result);
    EXPECT_EQ(expected_yaml, result_yaml);
  }

  RosExtrinsics::Config extrinsics_config;
  config.extrinsics = extrinsics_config;

  {  // ros without a frame should not be valid
    VirtualSensor sensor(config);
    const auto result = input::loadSensor(sensor, 0);
    ASSERT_TRUE(result);
    EXPECT_EQ(result.getType(), "camera");

    Camera::Config expected = expected_intrinsics;
    expected.extrinsics = expected_extrinsics;
    const auto expected_yaml = getExportedConfig(expected);
    const auto result_yaml = getExportedConfig(result);
    EXPECT_EQ(expected_yaml, result_yaml);
  }
}

}  // namespace hydra
