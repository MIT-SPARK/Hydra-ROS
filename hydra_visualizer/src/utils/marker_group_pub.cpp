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
#include "hydra_visualizer/utils/marker_group_pub.h"

namespace hydra {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

MarkerGroupPub::MarkerGroupPub(const ros::NodeHandle& nh) : nh_(nh) {}

void MarkerGroupPub::publish(const std::string& name,
                             const std_msgs::Header& header,
                             const MarkerCallback& func) const {
  publish(name, header, [&func]() -> MarkerArray {
    MarkerArray msg;
    msg.markers.push_back(func());
    return msg;
  });
}

void MarkerGroupPub::publish(const std::string& name,
                             const std_msgs::Header& header,
                             const ArrayCallback& func) const {
  auto iter = pubs_.find(name);
  if (iter == pubs_.end()) {
    TrackedPublisher pub{{}, nh_.advertise<MarkerArray>(name, 1, true)};
    iter = pubs_.emplace(name, pub).first;
  }

  if (!iter->second.pub.getNumSubscribers()) {
    return;  // avoid doing computation if we don't need to publish
  }

  MarkerArray msg;
  iter->second.tracker.add(func(), msg);
  for (auto& marker : msg.markers) {
    marker.header = header;
  }
  iter->second.tracker.clearPrevious(header, msg);
  iter->second.pub.publish(msg);
}

}  // namespace hydra
