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
#include <hydra/reconstruction/reconstruction_module.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/utils/marker_group_pub.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace hydra {

struct ImageGroupPub;

class ReconstructionVisualizer : public ReconstructionModule::Sink {
 public:
  struct Config {
    std::string ns = "~reconstruction";
    double min_weight = 0.0;
    double max_weight = 10.0;
    double marker_alpha = 0.5;
    bool use_relative_height = true;
    double slice_height = 0.0;
    double min_observation_weight = 1.0e-5;
    visualizer::RangeColormap::Config colormap;
    visualizer::CategoricalColormap::Config label_colormap;
  } const config;

  explicit ReconstructionVisualizer(const Config& config);

  virtual ~ReconstructionVisualizer();

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns,
            const Eigen::Isometry3d& world_T_sensor,
            const TsdfLayer& tsdf,
            const ReconstructionOutput& msg) const override;

 protected:
  void publishLabelImage(const ReconstructionOutput& output) const;

  ros::NodeHandle nh_;
  MarkerGroupPub pubs_;
  std::unique_ptr<ImageGroupPub> image_pubs_;
  const visualizer::RangeColormap colormap_;
  const visualizer::CategoricalColormap label_colormap_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<ReconstructionModule::Sink,
                                     ReconstructionVisualizer,
                                     Config>("ReconstructionVisualizer");
};

void declare_config(ReconstructionVisualizer::Config& config);

}  // namespace hydra
