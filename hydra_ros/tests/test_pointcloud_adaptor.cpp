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
#include <hydra_ros/reconstruction/pointcloud_adaptor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hydra {

bool comparePoints(const CloudInputPacket& lhs, const CloudInputPacket& rhs) {
  if (lhs.points.size() != rhs.points.size() ||
      lhs.points.type() != rhs.points.type()) {
    LOG(ERROR) << "points incompatible!";
    return false;
  }

  if (lhs.colors.size() != rhs.colors.size() ||
      lhs.colors.type() != rhs.colors.type()) {
    LOG(ERROR) << "colors incompatible!";
    return false;
  }

  if (lhs.labels.size() != rhs.labels.size() ||
      lhs.labels.type() != rhs.labels.type()) {
    LOG(ERROR) << "points incompatible";
    return false;
  }

  if (lhs.labels.type() != CV_32SC1 || lhs.points.type() != CV_32FC3 ||
      lhs.colors.type() != CV_8UC3) {
    LOG(ERROR) << "invalid types";
    return false;
  }

  for (int r = 0; r < lhs.points.rows; ++r) {
    for (int c = 0; c < lhs.points.cols; ++c) {
      const auto& p_lhs = lhs.points.at<cv::Vec3f>(r, c);
      const auto& p_rhs = rhs.points.at<cv::Vec3f>(r, c);
      const auto& vec_lhs = Eigen::Vector3f(p_lhs[0], p_lhs[1], p_lhs[2]);
      const auto& vec_rhs = Eigen::Vector3f(p_rhs[0], p_rhs[1], p_rhs[2]);
      if (!vec_lhs.isApprox(vec_rhs)) {
        LOG(ERROR) << "points not equal";
        return false;
      }

      if (lhs.labels.at<int32_t>(r, c) != rhs.labels.at<int32_t>(r, c)) {
        LOG(ERROR) << "labels not equal";
        return false;
      }

      if (lhs.colors.at<cv::Vec3b>(r, c) != rhs.colors.at<cv::Vec3b>(r, c)) {
        LOG(ERROR) << "labels not equal";
        return false;
      }
    }
  }

  return true;
}

std::string showCloud(const CloudInputPacket& cloud) {
  std::stringstream out;
  out << "pointcloud: {" << std::endl;
  for (int r = 0; r < cloud.points.rows; ++r) {
    for (int c = 0; c < cloud.points.cols; ++c) {
      const auto& point = cloud.points.at<cv::Vec3f>(r, c);
      out << " - (" << r << ", " << c << "): pos: [" << std::setprecision(4) << point[0]
          << ", " << point[1] << ", " << point[2] << "]";

      if (r < cloud.colors.rows && c < cloud.colors.cols) {
        const auto& color = cloud.colors.at<cv::Vec3b>(r, c);
        out << ", color: [" << static_cast<int>(color[0]) << ", "
            << static_cast<int>(color[1]) << ", " << static_cast<int>(color[2]) << "]";
      } else {
        out << ", color: n/a";
      }

      if (r < cloud.labels.rows && c < cloud.labels.cols) {
        out << ", label: " << cloud.labels.at<int32_t>(r, c);
      } else {
        out << ", label: n/a";
      }

      out << std::endl;
    }
  }
  out << "}";
  return out.str();
}

void resizePacket(CloudInputPacket& packet, int rows, int cols) {
  packet.points = cv::Mat(rows, cols, CV_32FC3);
  packet.colors = cv::Mat(rows, cols, CV_8UC3);
  packet.labels = cv::Mat(rows, cols, CV_32SC1);
}

pcl::PointXYZRGBL makeTestPointXYZRGBL(int r, int c, CloudInputPacket& expected) {
  size_t i = r * expected.points.cols + c;
  pcl::PointXYZRGBL point;
  point.x = static_cast<float>(i);
  point.y = static_cast<float>(i + 1);
  point.z = static_cast<float>(i + 2);
  point.r = static_cast<uint8_t>(i + 3);
  point.g = static_cast<uint8_t>(i + 4);
  point.b = static_cast<uint8_t>(i + 5);
  point.a = static_cast<uint8_t>(i + 6);
  point.label = static_cast<uint32_t>(i + 7);

  expected.points.at<cv::Vec3f>(r, c) = {point.x, point.y, point.z};
  expected.colors.at<cv::Vec3b>(r, c) = {point.r, point.g, point.b};
  expected.labels.at<int32_t>(r, c) = point.label;
  return point;
}

pcl::PointXYZL makeTestPointXYZL(int r, int c, CloudInputPacket& expected) {
  size_t i = r * expected.points.cols + c;
  pcl::PointXYZL point;
  point.x = static_cast<float>(i);
  point.y = static_cast<float>(i + 1);
  point.z = static_cast<float>(i + 2);
  point.label = static_cast<uint32_t>(i + 3);

  expected.points.at<cv::Vec3f>(r, c) = {point.x, point.y, point.z};
  expected.colors.at<cv::Vec3b>(r, c) = {0, 0, 0};
  expected.labels.at<int32_t>(r, c) = point.label;
  return point;
}

TEST(PointcloudAdaptor, ConvertPointXYZRGBL) {
  CloudInputPacket expected(0);
  resizePacket(expected, 5, 2);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud(2, 5);
  for (int r = 0; r < expected.points.rows; ++r) {
    for (int c = 0; c < expected.points.cols; ++c) {
      cloud.at(c, r) = makeTestPointXYZRGBL(r, c, expected);
    }
  }

  pcl::PCLPointCloud2 serialized_cloud;
  pcl::toPCLPointCloud2(cloud, serialized_cloud);
  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(serialized_cloud, msg);

  CloudInputPacket result(0);
  fillPointcloudPacket(msg, result, true);
  EXPECT_TRUE(comparePoints(expected, result))
      << "expected: " << showCloud(expected) << ", result: " << showCloud(result);
}

TEST(PointcloudAdaptor, ConvertPointXYZL) {
  CloudInputPacket expected(0);
  resizePacket(expected, 5, 2);

  pcl::PointCloud<pcl::PointXYZL> cloud(2, 5);
  for (int r = 0; r < expected.points.rows; ++r) {
    for (int c = 0; c < expected.points.cols; ++c) {
      cloud.at(c, r) = makeTestPointXYZL(r, c, expected);
    }
  }

  pcl::PCLPointCloud2 serialized_cloud;
  pcl::toPCLPointCloud2(cloud, serialized_cloud);
  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(serialized_cloud, msg);

  CloudInputPacket result(0);
  fillPointcloudPacket(msg, result, true);
  EXPECT_TRUE(comparePoints(expected, result))
      << "expected: " << showCloud(expected) << ", result: " << showCloud(result);
}

}  // namespace hydra
