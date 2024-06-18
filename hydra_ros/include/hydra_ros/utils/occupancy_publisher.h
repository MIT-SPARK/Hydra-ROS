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
#include <config_utilities/factory.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <hydra/places/gvd_voxel.h>
#include <hydra/reconstruction/reconstruction_module.h>
#include <ros/ros.h>

namespace hydra {

class OccupancyPublisher {
 public:
  struct Config {
    bool use_relative_height = true;
    double slice_height = 0.0;
    size_t num_slices = 1;
    double min_observation_weight = 1.0e-6;
    double min_distance = 0.3;
    bool add_robot_footprint = false;
    Eigen::Vector3f footprint_min;
    Eigen::Vector3f footprint_max;
  } const config;

  OccupancyPublisher(const Config& config, const ros::NodeHandle& nh);

  virtual ~OccupancyPublisher();

  void publishTsdf(uint64_t timestamp_ns,
                   const Eigen::Isometry3d& world_T_sensor,
                   const TsdfLayer& tsdf) const;

  void publishGvd(uint64_t timestamp_ns,
                  const Eigen::Isometry3d& world_T_sensor,
                  const places::GvdLayer& gvd) const;

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

class TsdfOccupancyPublisher : public ReconstructionModule::Sink {
 public:
  struct Config {
    std::string ns = "~tsdf";
    OccupancyPublisher::Config extraction;
    bool collate = false;
  } const config;

  explicit TsdfOccupancyPublisher(const Config& config);

  virtual ~TsdfOccupancyPublisher() = default;

  void call(uint64_t timestamp_ns,
            const Eigen::Isometry3d& world_T_sensor,
            const TsdfLayer& tsdf,
            const ReconstructionOutput& msg) const override;

 private:
  OccupancyPublisher pub_;
  mutable TsdfLayer::Ptr tsdf_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<ReconstructionModule::Sink,
                                     TsdfOccupancyPublisher,
                                     Config>("TsdfOccupancyPublisher");
};

class GvdOccupancyPublisher : public GvdPlaceExtractor::Sink {
 public:
  struct Config {
    std::string ns = "~gvd";
    OccupancyPublisher::Config extraction;
    bool collate = false;
  } const config;

  explicit GvdOccupancyPublisher(const Config& config);

  virtual ~GvdOccupancyPublisher() = default;

  void call(uint64_t timestamp_ns,
            const Eigen::Isometry3f& world_T_sensor,
            const places::GvdLayer& gvd,
            const places::GraphExtractorInterface* extractor) const override;

 private:
  OccupancyPublisher pub_;
  mutable places::GvdLayer::Ptr gvd_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<GvdPlaceExtractor::Sink,
                                     GvdOccupancyPublisher,
                                     Config>("GvdOccupancyPublisher");
};

void declare_config(OccupancyPublisher::Config& config);
void declare_config(GvdOccupancyPublisher::Config& config);
void declare_config(TsdfOccupancyPublisher::Config& config);

}  // namespace hydra
