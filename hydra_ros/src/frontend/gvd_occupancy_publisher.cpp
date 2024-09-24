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
#include "hydra_ros/frontend/gvd_occupancy_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

namespace hydra {
namespace voxel_traits {

template <>
bool isObserved(const places::GvdVoxel& voxel, float) {
  return voxel.observed;
}

template <>
float getDistance(const places::GvdVoxel& voxel) {
  return voxel.distance;
}

}  // namespace voxel_traits

void declare_config(GvdOccupancyPublisher::Config& config) {
  using namespace config;
  name("GvdOccupancyPublisher::Config");
  base<OccupancyPublisherConfig>(config);
  field(config.ns, "ns");
}

GvdOccupancyPublisher::GvdOccupancyPublisher(const Config& config)
    : OccupancyPublisher<places::GvdBlock>(config, ros::NodeHandle(config.ns)),
      config(config::checkValid(config)) {}

std::string GvdOccupancyPublisher::printInfo() const {
  return config::toString(config);
}

void GvdOccupancyPublisher::call(uint64_t timestamp_ns,
                                 const Eigen::Isometry3f& world_T_body,
                                 const places::GvdLayer& gvd,
                                 const places::GraphExtractorInterface*) const {
  publish(timestamp_ns, world_T_body.cast<double>(), gvd);
}

namespace {

static const auto registration_ =
    config::RegistrationWithConfig<GvdPlaceExtractor::Sink,
                                   GvdOccupancyPublisher,
                                   GvdOccupancyPublisher::Config>(
        "GvdOccupancyPublisher");

}
}  // namespace hydra
