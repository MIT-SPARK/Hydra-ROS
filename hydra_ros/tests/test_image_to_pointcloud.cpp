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
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/common/hydra_config.h>
#include <hydra_ros/utils/image_to_pointcloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace hydra {

sensor_msgs::Image::ConstPtr getImageMessage(const cv::Mat& img,
                                             const std::string& encoding) {
  std_msgs::Header header;
  cv_bridge::CvImage bridge_img(header, encoding, img);
  return bridge_img.toImageMsg();
}

struct TestData {
  TestData(int rows,
           int cols,
           int rgb_type = CV_8UC3,
           bool depth_in_mm = false,
           bool rgb_order = true,
           int label_type = CV_32SC1) {
    cv::Mat rgb(rows, cols, rgb_type);
    cv::Mat depth(rows, cols, depth_in_mm ? CV_16UC1 : CV_32FC1);
    cv::Mat labels(rows, cols, label_type);

    for (int row = 0; row < rgb.rows; ++row) {
      for (int col = 0; col < rgb.cols; ++col) {
        const uint8_t color_value =
            row * rgb.cols * rgb.channels() + col * rgb.channels();
        uint8_t* rgb_ptr = rgb.ptr<uint8_t>(row, col);
        rgb_ptr[0] = color_value;
        rgb_ptr[1] = color_value + 1;
        rgb_ptr[2] = color_value + 2;
        if (rgb.channels() == 4) {
          rgb_ptr[3] = color_value + 3;
        }

        size_t index = row * rgb.cols + col;
        if (depth_in_mm) {
          depth.at<uint16_t>(row, col) = index;
        } else {
          depth.at<float>(row, col) =
              index == 0 ? std::numeric_limits<float>::infinity() : index;
        }

        if (label_type == CV_32SC1) {
          labels.at<int32_t>(row, col) = index + 2;
        } else if (label_type == CV_16UC1) {
          labels.at<uint16_t>(row, col) = index + 2;
        } else if (label_type == CV_8UC1) {
          labels.at<uint8_t>(row, col) = index + 2;
        } else {
          throw std::domain_error("invalid label type");
        }
      }
    }

    std::string rgb_encoding;
    if (rgb_order) {
      rgb_encoding = rgb.channels() == 3 ? sensor_msgs::image_encodings::RGB8
                                         : sensor_msgs::image_encodings::RGBA8;
    } else {
      rgb_encoding = rgb.channels() == 3 ? sensor_msgs::image_encodings::BGR8
                                         : sensor_msgs::image_encodings::BGRA8;
    }
    color_msg = getImageMessage(rgb, rgb_encoding);
    depth_msg = getImageMessage(depth,
                                depth_in_mm ? sensor_msgs::image_encodings::TYPE_16UC1
                                            : sensor_msgs::image_encodings::TYPE_32FC1);

    std::string label_encoding;
    if (label_type == CV_32SC1) {
      label_encoding = sensor_msgs::image_encodings::TYPE_32SC1;
    } else if (label_type == CV_16UC1) {
      label_encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    } else if (label_type == CV_8UC1) {
      label_encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    } else {
      throw std::domain_error("invalid label type");
    }

    labels_msg = getImageMessage(labels, label_encoding);
  }

  sensor_msgs::Image::ConstPtr color_msg;
  sensor_msgs::Image::ConstPtr depth_msg;
  sensor_msgs::Image::ConstPtr labels_msg;
};

sensor_msgs::Image::ConstPtr makeColorImage(int rows,
                                            int cols,
                                            std::vector<std::array<uint8_t, 3>>& colors,
                                            int rgb_type = CV_8UC3,
                                            bool rgb_order = true) {
  cv::Mat rgb(rows, cols, rgb_type);

  for (int row = 0; row < rgb.rows; ++row) {
    for (int col = 0; col < rgb.cols; ++col) {
      size_t index = row * rgb.cols + col;
      const auto& color = colors.at(index % colors.size());
      uint8_t* rgb_ptr = rgb.ptr<uint8_t>(row, col);
      rgb_ptr[0] = color[0];
      rgb_ptr[1] = color[1];
      rgb_ptr[2] = color[2];
      if (rgb.channels() == 4) {
        rgb_ptr[3] = 255;
      }
    }
  }

  std::string rgb_encoding;
  if (rgb_order) {
    rgb_encoding = rgb.channels() == 3 ? sensor_msgs::image_encodings::RGB8
                                       : sensor_msgs::image_encodings::RGBA8;
  } else {
    rgb_encoding = rgb.channels() == 3 ? sensor_msgs::image_encodings::BGR8
                                       : sensor_msgs::image_encodings::BGRA8;
  }
  return getImageMessage(rgb, rgb_encoding);
}

pcl::PointXYZRGBL makePoint(float x,
                            float y,
                            float z,
                            uint8_t r,
                            uint8_t g,
                            uint8_t b,
                            uint8_t a,
                            uint32_t label) {
  pcl::PointXYZRGBL point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.r = r;
  point.g = g;
  point.b = b;
  point.a = a;
  point.label = label;
  return point;
}

pcl::PointXYZRGBL makePoint(
    uint8_t r, uint8_t g, uint8_t b, uint8_t a, uint32_t label) {
  return makePoint(std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::quiet_NaN(),
                   r,
                   g,
                   b,
                   a,
                   label);
}

void comparePoints(const pcl::PointXYZRGBL& lhs, const pcl::PointXYZRGBL& rhs) {
  if (std::isnan(lhs.x)) {
    EXPECT_TRUE(std::isnan(rhs.x));
    EXPECT_TRUE(std::isnan(rhs.y));
    EXPECT_TRUE(std::isnan(rhs.z));
    EXPECT_TRUE(std::isnan(lhs.y));
    EXPECT_TRUE(std::isnan(lhs.z));
  } else {
    EXPECT_NEAR(lhs.x, rhs.x, 1.0e-9f);
    EXPECT_NEAR(lhs.y, rhs.y, 1.0e-9f);
    EXPECT_NEAR(lhs.z, rhs.z, 1.0e-9f);
  }
  EXPECT_EQ(static_cast<int>(lhs.r), static_cast<int>(rhs.r));
  EXPECT_EQ(static_cast<int>(lhs.g), static_cast<int>(rhs.g));
  EXPECT_EQ(static_cast<int>(lhs.b), static_cast<int>(rhs.b));
  EXPECT_EQ(static_cast<int>(lhs.a), static_cast<int>(rhs.a));
  EXPECT_EQ(lhs.label, rhs.label);
}

TEST(PointcloudAdaptor, ConvertImageToPointclouds) {
  TestData data(2, 3);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(0, 1, 2, 255, 2));
  expected.push_back(makePoint(0.5f, -0.5f, 1.0f, 3, 4, 5, 255, 3));
  expected.push_back(makePoint(3.0f, -1.0f, 2.0f, 6, 7, 8, 255, 4));
  expected.push_back(makePoint(-1.5f, 0.0f, 3.0f, 9, 10, 11, 255, 5));
  expected.push_back(makePoint(2.0f, 0.0f, 4.0f, 12, 13, 14, 255, 6));
  expected.push_back(makePoint(7.5f, 0.0f, 5.0f, 15, 16, 17, 255, 7));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

TEST(PointcloudAdaptor, ConvertImageToPointcloudsWithAlpha) {
  TestData data(2, 3, CV_8UC4);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(0, 1, 2, 3, 2));
  expected.push_back(makePoint(0.5f, -0.5f, 1.0f, 4, 5, 6, 7, 3));
  expected.push_back(makePoint(3.0f, -1.0f, 2.0f, 8, 9, 10, 11, 4));
  expected.push_back(makePoint(-1.5f, 0.0f, 3.0f, 12, 13, 14, 15, 5));
  expected.push_back(makePoint(2.0f, 0.0f, 4.0f, 16, 17, 18, 19, 6));
  expected.push_back(makePoint(7.5f, 0.0f, 5.0f, 20, 21, 22, 23, 7));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

TEST(PointcloudAdaptor, ConvertImageToPointcloudsWithAlphaAndMM) {
  TestData data(2, 3, CV_8UC4, true);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(0, 1, 2, 3, 2));
  expected.push_back(makePoint(0.0005f, -0.0005f, 0.001f, 4, 5, 6, 7, 3));
  expected.push_back(makePoint(0.003f, -0.001f, 0.002f, 8, 9, 10, 11, 4));
  expected.push_back(makePoint(-0.0015f, 0.0f, 0.003f, 12, 13, 14, 15, 5));
  expected.push_back(makePoint(0.002f, 0.0f, 0.004f, 16, 17, 18, 19, 6));
  expected.push_back(makePoint(0.0075f, 0.0f, 0.005f, 20, 21, 22, 23, 7));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

TEST(PointcloudAdaptor, ConvertImageToPointcloudsBgr) {
  TestData data(2, 3, CV_8UC3, false, false);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(2, 1, 0, 255, 2));
  expected.push_back(makePoint(0.5f, -0.5f, 1.0f, 5, 4, 3, 255, 3));
  expected.push_back(makePoint(3.0f, -1.0f, 2.0f, 8, 7, 6, 255, 4));
  expected.push_back(makePoint(-1.5f, 0.0f, 3.0f, 11, 10, 9, 255, 5));
  expected.push_back(makePoint(2.0f, 0.0f, 4.0f, 14, 13, 12, 255, 6));
  expected.push_back(makePoint(7.5f, 0.0f, 5.0f, 17, 16, 15, 255, 7));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

TEST(PointcloudAdaptor, ConvertImageToPointcloudsBgrWithAlpha) {
  TestData data(2, 3, CV_8UC4, false, false);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(2, 1, 0, 3, 2));
  expected.push_back(makePoint(0.5f, -0.5f, 1.0f, 6, 5, 4, 7, 3));
  expected.push_back(makePoint(3.0f, -1.0f, 2.0f, 10, 9, 8, 11, 4));
  expected.push_back(makePoint(-1.5f, 0.0f, 3.0f, 14, 13, 12, 15, 5));
  expected.push_back(makePoint(2.0f, 0.0f, 4.0f, 18, 17, 16, 19, 6));
  expected.push_back(makePoint(7.5f, 0.0f, 5.0f, 22, 21, 20, 23, 7));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

TEST(PointcloudAdaptor, BadColorEncoding) {
  TestData data(2, 3, CV_8UC4, false, false);
  const_cast<sensor_msgs::Image*>(data.color_msg.get())->encoding = "32_FC1";

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  bool valid = fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);
  EXPECT_TRUE(!valid);
}

TEST(PointcloudAdaptor, ConvertImageToPointclouds16BitLabel) {
  TestData data(2, 3, CV_8UC4, false, false, CV_16UC1);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(2, 1, 0, 3, 2));
  expected.push_back(makePoint(0.5f, -0.5f, 1.0f, 6, 5, 4, 7, 3));
  expected.push_back(makePoint(3.0f, -1.0f, 2.0f, 10, 9, 8, 11, 4));
  expected.push_back(makePoint(-1.5f, 0.0f, 3.0f, 14, 13, 12, 15, 5));
  expected.push_back(makePoint(2.0f, 0.0f, 4.0f, 18, 17, 16, 19, 6));
  expected.push_back(makePoint(7.5f, 0.0f, 5.0f, 22, 21, 20, 23, 7));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

struct HydraConfigGuard {
  ~HydraConfigGuard() { HydraConfig::reset(); }
};

TEST(PointcloudAdaptor, ConvertImageToPointcloudsColorLabel) {
  TestData data(2, 3, CV_8UC4, false, false, CV_16UC1);

  HydraConfigGuard guard;
  struct LabelSpaceConfig config;
  ros::NodeHandle nh;
  nh.getParam("/color_label_conversion/semantic_map", config.colormap_filepath);
  HydraConfig::instance().setLabelSpaceConfig(config);

  std::vector<std::array<uint8_t, 3>> colors{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
  data.labels_msg = makeColorImage(2, 3, colors);

  sensor_msgs::CameraInfo info;
  info.K = {1.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0};

  sensor_msgs::PointCloud2 msg;
  fillPointcloudFromImages(
      *data.color_msg, *data.depth_msg, *data.labels_msg, info, msg);

  pcl::PCLPointCloud2 serialized_cloud;
  pcl_conversions::toPCL(msg, serialized_cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> cloud;
  pcl::fromPCLPointCloud2(serialized_cloud, cloud);

  pcl::PointCloud<pcl::PointXYZRGBL> expected;
  expected.push_back(makePoint(2, 1, 0, 3, 0));
  expected.push_back(makePoint(0.5f, -0.5f, 1.0f, 6, 5, 4, 7, 1));
  expected.push_back(makePoint(3.0f, -1.0f, 2.0f, 10, 9, 8, 11, 2));
  expected.push_back(makePoint(-1.5f, 0.0f, 3.0f, 14, 13, 12, 15, 0));
  expected.push_back(makePoint(2.0f, 0.0f, 4.0f, 18, 17, 16, 19, 1));
  expected.push_back(makePoint(7.5f, 0.0f, 5.0f, 22, 21, 20, 23, 2));

  ASSERT_EQ(cloud.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("comparing pointclouds at point " + std::to_string(i));
    comparePoints(cloud[i], expected[i]);
  }
}

}  // namespace hydra
