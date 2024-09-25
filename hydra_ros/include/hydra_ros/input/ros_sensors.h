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
#include <hydra/input/camera.h>
#include <hydra/input/sensor.h>

namespace ros {
class NodeHandle;
}

namespace rosbag {
class Bag;
}

namespace hydra {

struct InvalidSensor : Sensor {
  InvalidSensor() : Sensor(Sensor::Config(), "") {}
  virtual ~InvalidSensor() = default;
  float getPointDepth(const Eigen::Vector3f&) const override { return 0.0; }
  float computeRayDensity(float, float) const override { return 0.0; }
  bool finalizeRepresentations(InputData&, bool) const override { return false; }
  bool projectPointToImagePlane(const Eigen::Vector3f&, float&, float&) const override {
    return false;
  }
  bool projectPointToImagePlane(const Eigen::Vector3f&, int&, int&) const override {
    return false;
  }
  bool pointIsInViewFrustum(const Eigen::Vector3f&, float) const override {
    return false;
  };
};

struct RosExtrinsics : SensorExtrinsics {
  struct Config {
    std::string sensor_frame = "";
  } const config;

  explicit RosExtrinsics(const Config& config);

 private:
  inline static const auto r_ =
      config::RegistrationWithConfig<SensorExtrinsics, RosExtrinsics, Config>("ros");
};

struct RosCamera : InvalidSensor {
  struct Config : Sensor::Config {
    std::string ns = "";
  } const config;

  explicit RosCamera(const Config& config);

 private:
  inline static const auto r_ =
      config::RegistrationWithConfig<Sensor, RosCamera, Config>("camera_info");
};

struct RosbagCamera : InvalidSensor {
  struct Config : Sensor::Config {
    std::string topic = "";
  } const config;

  explicit RosbagCamera(const Config& config);

 private:
  inline static const auto r_ =
      config::RegistrationWithConfig<Sensor, RosbagCamera, Config>(
          "rosbag_camera_info");
};

void declare_config(RosExtrinsics::Config& config);

void declare_config(RosCamera::Config& config);

void declare_config(RosbagCamera::Config& config);

namespace input {

config::VirtualConfig<Sensor> loadSensor(const config::VirtualConfig<Sensor>& sensor,
                                         const std::string& name);

config::VirtualConfig<Sensor> loadSensor(const rosbag::Bag& bag,
                                         const config::VirtualConfig<Sensor>& sensor);

}  // namespace input

}  // namespace hydra
