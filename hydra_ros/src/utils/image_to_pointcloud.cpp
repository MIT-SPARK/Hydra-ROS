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
#include "hydra_ros/utils/image_to_pointcloud.h"

#include <glog/logging.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "hydra_ros/utils/image_color_adaptor.h"
#include "hydra_ros/utils/image_depth_adaptor.h"
#include "hydra_ros/utils/image_label_adaptor.h"

namespace hydra {

struct Camera {
  explicit Camera(const sensor_msgs::CameraInfo& info)
      : cx(info.K[2]),
        cy(info.K[5]),
        fx(info.K[0]),
        fy(info.K[4]),
        fx_inv(1.0f / info.K[0]),
        fy_inv(1.0f / info.K[4]) {}

  float unproject_u(float u, float depth_m) const {
    return (u - cx) * depth_m * fx_inv;
  }
  float unproject_v(float v, float depth_m) const {
    return (v - cy) * depth_m * fy_inv;
  }

  const float cx;
  const float cy;
  const float fx;
  const float fy;
  const float fx_inv;
  const float fy_inv;
};

struct CloudIter {
  explicit CloudIter(sensor_msgs::PointCloud2& cloud)
      : pos_(cloud, "x"),
        r_(cloud, "r"),
        g_(cloud, "g"),
        b_(cloud, "b"),
        a_(cloud, "a"),
        label_(cloud, "label") {}

  void setPos(float x, float y, float z) {
    pos_[0] = x;
    pos_[1] = y;
    pos_[2] = z;
  }

  void setPosInvalid() {
    pos_[0] = std::numeric_limits<float>::quiet_NaN();
    pos_[1] = std::numeric_limits<float>::quiet_NaN();
    pos_[2] = std::numeric_limits<float>::quiet_NaN();
  }

  void setColor(const std::array<uint8_t, 4>& rgba) {
    *r_ = rgba[0];
    *g_ = rgba[1];
    *b_ = rgba[2];
    *a_ = rgba[3];
  }

  void setLabel(uint32_t label) { *label_ = label; }

  void increment() {
    ++pos_;
    ++r_;
    ++g_;
    ++b_;
    ++a_;
    ++label_;
  }

  sensor_msgs::PointCloud2Iterator<float> pos_;
  sensor_msgs::PointCloud2Iterator<uint8_t> r_;
  sensor_msgs::PointCloud2Iterator<uint8_t> g_;
  sensor_msgs::PointCloud2Iterator<uint8_t> b_;
  sensor_msgs::PointCloud2Iterator<uint8_t> a_;
  sensor_msgs::PointCloud2Iterator<uint32_t> label_;
};

bool fillPointcloudFromImages(const sensor_msgs::Image& color,
                              const sensor_msgs::Image& depth,
                              const sensor_msgs::Image& labels,
                              const sensor_msgs::CameraInfo& info,
                              sensor_msgs::PointCloud2& cloud) {
  cloud.width = color.width;
  cloud.height = color.height;
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  // clang-format off
  sensor_msgs::PointCloud2Modifier cloud_adaptor(cloud);
  cloud_adaptor.setPointCloud2Fields(5,
                                     "x", 1, sensor_msgs::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::PointField::FLOAT32,
                                     "rgb", 1, sensor_msgs::PointField::FLOAT32,
                                     "label", 1, sensor_msgs::PointField::UINT32);
  // clang-format on

  CloudIter iter(cloud);
  const Camera camera(info);

  // TODO(nathan) this try-catch is lazy
  try {
    const ColorAdaptor color_adaptor(color);
    const DepthAdaptor depth_adaptor(depth);
    const LabelAdaptor label_adaptor(labels);
    for (uint32_t v = 0; v < cloud.height; ++v) {
      for (uint32_t u = 0; u < cloud.width; ++u) {
        const auto depth_value = depth_adaptor.read(depth, v, u);
        if (!depth_value) {
          iter.setPosInvalid();
        } else {
          const auto d_m = *depth_value;
          iter.setPos(camera.unproject_u(u, d_m), camera.unproject_v(v, d_m), d_m);
        }

        iter.setColor(color_adaptor.read(color, v, u));
        iter.setLabel(label_adaptor.read(labels, v, u));
        iter.increment();
      }
    }
  } catch (std::exception& e) {
    LOG(ERROR) << e.what();
    return false;
  }

  return true;
}

}  // namespace hydra
