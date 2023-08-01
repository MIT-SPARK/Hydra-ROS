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
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra_ros/utils/pointcloud_adaptor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace voxblox {

bool operator==(const Color& lhs, const Color& rhs) {
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b && lhs.a == rhs.a;
}

std::ostream& operator<<(std::ostream& out, const Color& c) {
  return out << "[" << static_cast<int>(c.r) << ", " << static_cast<int>(c.g) << ", "
             << static_cast<int>(c.b) << ", " << static_cast<int>(c.a) << "]";
}

}  // namespace voxblox

namespace hydra {

struct VoxbloxCloud {
  voxblox::Pointcloud points;
  voxblox::Colors colors;
  std::vector<uint32_t> labels;
};

bool operator==(const VoxbloxCloud& lhs, const VoxbloxCloud& rhs) {
  if (lhs.labels != rhs.labels) {
    LOG(ERROR) << "labels inequal!";
    return false;
  }

  if (lhs.colors != rhs.colors) {
    LOG(ERROR) << "colors inequal!";
    return false;
  }

  if (lhs.points.size() != rhs.points.size()) {
    LOG(ERROR) << "points not same size!";
    return false;
  }

  for (size_t i = 0; i < lhs.points.size(); ++i) {
    if ((lhs.points.at(i) - rhs.points.at(i)).norm() > 1.0e-9) {
      LOG(ERROR) << "points not equal";
      return false;
    }
  }

  return true;
}

std::ostream& operator<<(std::ostream& out, const VoxbloxCloud& cloud) {
  out << "pointcloud: {" << std::endl;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& point = cloud.points.at(i);
    out << " - " << i << ": pos: [" << std::setprecision(4) << point.x() << ", "
        << point.y() << ", " << point.z() << "]";

    if (i < cloud.colors.size()) {
      out << ", color: " << cloud.colors.at(i);
    } else {
      out << ", color: n/a";
    }

    if (i < cloud.labels.size()) {
      out << ", label: " << cloud.labels.at(i);
    } else {
      out << ", label: n/a";
    }

    out << std::endl;
  }
  out << "}";
  return out;
}

pcl::PointXYZRGBL makeTestPointXYZRGBL(size_t i, VoxbloxCloud& expected) {
  pcl::PointXYZRGBL point;
  point.x = static_cast<float>(i);
  point.y = static_cast<float>(i + 1);
  point.z = static_cast<float>(i + 2);
  point.r = static_cast<uint8_t>(i + 3);
  point.g = static_cast<uint8_t>(i + 4);
  point.b = static_cast<uint8_t>(i + 5);
  point.a = static_cast<uint8_t>(i + 6);
  point.label = static_cast<uint32_t>(i + 7);

  expected.points.push_back(voxblox::Point(point.x, point.y, point.z));
  expected.colors.push_back(voxblox::Color(point.r, point.g, point.b, point.a));
  expected.labels.push_back(point.label);
  return point;
}

pcl::PointXYZL makeTestPointXYZL(size_t i, VoxbloxCloud& expected) {
  pcl::PointXYZL point;
  point.x = static_cast<float>(i);
  point.y = static_cast<float>(i + 1);
  point.z = static_cast<float>(i + 2);
  point.label = static_cast<uint32_t>(i + 3);

  expected.points.push_back(voxblox::Point(point.x, point.y, point.z));
  expected.colors.push_back(voxblox::Color());
  expected.labels.push_back(point.label);
  return point;
}

TEST(PointcloudAdaptor, ConvertPointXYZRGBL) {
  VoxbloxCloud expected;

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  for (size_t i = 0; i < 10; ++i) {
    cloud.push_back(makeTestPointXYZRGBL(i, expected));
  }

  pcl::PCLPointCloud2 serialized_cloud;
  pcl::toPCLPointCloud2(cloud, serialized_cloud);
  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(serialized_cloud, msg);

  VoxbloxCloud result;
  fillVoxbloxPointcloud(msg, result.points, result.colors, result.labels, true);

  EXPECT_EQ(expected, result);
}

TEST(PointcloudAdaptor, ConvertPointXYZL) {
  VoxbloxCloud expected;

  pcl::PointCloud<pcl::PointXYZL> cloud;
  for (size_t i = 0; i < 10; ++i) {
    cloud.push_back(makeTestPointXYZL(i, expected));
  }

  pcl::PCLPointCloud2 serialized_cloud;
  pcl::toPCLPointCloud2(cloud, serialized_cloud);
  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(serialized_cloud, msg);

  VoxbloxCloud result;
  fillVoxbloxPointcloud(msg, result.points, result.colors, result.labels, true);

  EXPECT_EQ(expected, result);
}

}  // namespace hydra
