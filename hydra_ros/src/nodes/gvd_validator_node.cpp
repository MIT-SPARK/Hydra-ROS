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
#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/reconstruction/camera.h>
#include <hydra/reconstruction/combo_integrator.h>
#include <hydra/reconstruction/projective_integrator.h>
#include <hydra/reconstruction/reconstruction_config.h>
#include <hydra/reconstruction/sensor.h>
#include <hydra/reconstruction/volumetric_map.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include <voxblox/core/common.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>

#include <Eigen/Dense>

#include "hydra_ros/visualizer/places_visualizer.h"

DEFINE_string(config, "", "gvd integrator yaml config");
DEFINE_string(output_path, "", "output directory");
DEFINE_int32(max_updates, 0, "maximum number of updates per bag");
DEFINE_int32(voxels_per_side, 16, "voxel block size");
DEFINE_double(voxel_size, 0.1, "voxel size");

namespace hydra {

using places::GvdVoxel;
using sensor_msgs::Image;
using voxblox::BlockIndexList;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
using TimeSync = message_filters::Synchronizer<Policy>;
using TsdfConfig = ProjectiveIntegratorConfig;
using MeshConfig = hydra::MeshIntegratorConfig;
using GvdConfig = hydra::places::GvdIntegratorConfig;

struct BagConfig {
  std::string color_topic;
  std::string depth_topic;
  double start = -1.0;
  double duration = -1.0;
  std::vector<float> intrinsics;
  int image_width = 640;
  int image_height = 480;
};

void declare_config(BagConfig& conf) {
  using namespace config;
  name("BagConfig");
  field(conf.color_topic, "color_topic");
  field(conf.depth_topic, "depth_topic");
  field(conf.start, "start");
  field(conf.duration, "duration");
  field(conf.intrinsics, "intrinsics");
  field(conf.image_width, "image_width");
  field(conf.image_height, "image_height");
}

struct ComparisonResult {
  size_t num_missing_lhs = 0;
  size_t num_missing_rhs = 0;
  size_t num_observed = 0;
  size_t num_unobserved = 0;
  size_t num_rhs_seen_lhs_unseen = 0;
  size_t num_lhs_seen_rhs_unseen = 0;
  size_t num_fixed_different = 0;
  size_t num_parent_different = 0;
  size_t num_basis_different = 0;
  size_t num_surface_different = 0;
  double min_error = std::numeric_limits<double>::infinity();
  double max_error = 0.0;
};

std::ostream& operator<<(std::ostream& out, const ComparisonResult& r) {
  return out << "  - " << r.num_missing_lhs << " / " << r.num_missing_rhs
             << " unallocated (lhs / rhs) " << std::endl
             << "  - " << r.num_lhs_seen_rhs_unseen << " / "
             << r.num_rhs_seen_lhs_unseen << " unique (lhs / rhs)" << std::endl
             << "  - observed:   " << r.num_observed << std::endl
             << "  - unobserved: " << r.num_unobserved << std::endl
             << "  - fixed: " << r.num_fixed_different << std::endl
             << "  - parent: " << r.num_parent_different << std::endl
             << "  - basis: " << r.num_basis_different << std::endl
             << "  - surface: " << r.num_surface_different << std::endl
             << "  - error: [" << r.min_error << ", " << r.max_error << "]";
}

size_t getMissingBlocks(const Layer<GvdVoxel>& layer,
                        const BlockIndexList blocks,
                        const Layer<GvdVoxel>& other_layer) {
  size_t num_missing = 0;
  for (const auto& idx : blocks) {
    if (other_layer.hasBlock(idx)) {
      continue;
    }

    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      if (block.getVoxelByLinearIndex(i).observed) {
        num_missing++;
      }
    }
  }
  return num_missing;
}

ComparisonResult compareLayers(const Layer<GvdVoxel>& lhs, const Layer<GvdVoxel>& rhs) {
  voxblox::BlockIndexList lhs_blocks;
  lhs.getAllAllocatedBlocks(&lhs_blocks);

  voxblox::BlockIndexList rhs_blocks;
  rhs.getAllAllocatedBlocks(&rhs_blocks);

  ComparisonResult results;
  results.num_missing_lhs = getMissingBlocks(rhs, rhs_blocks, lhs);
  results.num_missing_rhs = getMissingBlocks(lhs, lhs_blocks, rhs);

  for (const auto& idx : lhs_blocks) {
    if (!rhs.hasBlock(idx)) {
      continue;
    }

    const auto& lhs_block = lhs.getBlockByIndex(idx);
    const auto& rhs_block = rhs.getBlockByIndex(idx);
    for (size_t i = 0; i < lhs_block.num_voxels(); ++i) {
      const GvdVoxel& lhs_voxel = lhs_block.getVoxelByLinearIndex(i);
      const GvdVoxel& rhs_voxel = rhs_block.getVoxelByLinearIndex(i);
      if (!lhs_voxel.observed && !rhs_voxel.observed) {
        results.num_unobserved++;
        continue;
      }

      results.num_observed++;

      if (!lhs_voxel.observed || !rhs_voxel.observed) {
        results.num_rhs_seen_lhs_unseen += (!lhs_voxel.observed ? 1 : 0);
        results.num_lhs_seen_rhs_unseen += (!rhs_voxel.observed ? 1 : 0);
        continue;
      }

      results.num_fixed_different += (lhs_voxel.fixed != rhs_voxel.fixed) ? 1 : 0;
      results.num_parent_different +=
          (lhs_voxel.has_parent != rhs_voxel.has_parent) ? 1 : 0;
      results.num_basis_different +=
          (lhs_voxel.num_extra_basis != rhs_voxel.num_extra_basis) ? 1 : 0;
      results.num_surface_different +=
          (lhs_voxel.on_surface != rhs_voxel.on_surface) ? 1 : 0;

      double error = std::abs(lhs_voxel.distance - rhs_voxel.distance);
      results.min_error = std::min(results.min_error, error);
      results.max_error = std::max(results.max_error, error);
    }
  }

  return results;
}

bool gvdFlagsSame(const GvdVoxel& lhs, const GvdVoxel& rhs) {
  return lhs.fixed == rhs.fixed && lhs.has_parent == rhs.has_parent &&
         lhs.num_extra_basis == rhs.num_extra_basis && lhs.on_surface == rhs.on_surface;
}

struct GvdValidator {
  struct Intrinstics {
    float fx;
    float fy;
    float cx;
    float cy;
  } intrinsics;

  GvdValidator(const config::VirtualConfig<Sensor>& sensor_config,
               const VolumetricMap::Config& map_config,
               const TsdfConfig& tsdf_config,
               const MeshConfig& mesh_config,
               const GvdConfig& gvd_config,
               const ComboIntegrator::GraphExtractorConfig& graph_config)
      : gvd_config(gvd_config), mesh_config(mesh_config), graph_config(graph_config) {
    // roughly 3000 hours: should be enough to store all the tfs
    ros::Duration max_duration;
    max_duration.fromSec(1.0e7);
    buffer.reset(new tf2::BufferCore(max_duration));

    visualizer.reset(new PlacesVisualizer("~"));
    full_visualizer.reset(new PlacesVisualizer("~full"));

    map.reset(new VolumetricMap(map_config));
    tsdf_integrator.reset(new ProjectiveIntegrator(tsdf_config));

    gvd.reset(new Layer<GvdVoxel>(map->voxel_size(), map->voxels_per_side()));
    gvd_integrator.reset(
        new ComboIntegrator(gvd_config, gvd, &mesh_config, graph_config));

    sensor = sensor_config.create();
  }

  void addTfsFromBag(const rosbag::Bag& bag) {
    std::vector<std::string> tf_topic{"/tf"};
    rosbag::View view(bag, rosbag::TopicQuery(tf_topic));
    for (const auto& m : view) {
      const auto msg = m.instantiate<tf2_msgs::TFMessage>();
      if (!msg) {
        LOG(ERROR) << "Invalid message on /tf topic!";
        continue;
      }

      for (const auto& transform : msg->transforms) {
        buffer->setTransform(transform, "rosbag");
      }
    }

    std::vector<std::string> static_tf_topic{"/tf_static"};
    rosbag::View static_view(bag, rosbag::TopicQuery(static_tf_topic));
    for (const auto& m : static_view) {
      const auto msg = m.instantiate<tf2_msgs::TFMessage>();
      if (!msg) {
        LOG(ERROR) << "Invalid message on /tf topic!";
        continue;
      }

      for (const auto& transform : msg->transforms) {
        buffer->setTransform(transform, "rosbag", true);
      }
    }
  }

  void fillPointCloud(const cv::Mat& color,
                      const cv::Mat& depth,
                      voxblox::Pointcloud& xyz,
                      voxblox::Colors& colors) {
    for (int r = 0; r < color.rows; ++r) {
      for (int c = 0; c < color.cols; ++c) {
        const ssize_t index = r * color.cols + c;
        const float u = (c - intrinsics.cx) / intrinsics.fx;
        const float v = (r - intrinsics.cy) / intrinsics.fy;

        Eigen::Vector3f& point = xyz[index];
        const float depth_value = depth.at<float>(r, c);
        point.x() = u * depth_value;
        point.y() = v * depth_value;
        point.z() = depth_value;

        auto& color_point = colors[index];
        const auto& pixel = color.at<cv::Vec3b>(r, c);
        color_point.r = pixel[0];
        color_point.g = pixel[1];
        color_point.b = pixel[2];
      }
    }
  }

  void handleImages(const sensor_msgs::Image::ConstPtr& rgb_msg,
                    const sensor_msgs::Image::ConstPtr& depth_msg) {
    ++num_updates;

    auto rgb = cv_bridge::toCvCopy(rgb_msg);
    if (rgb->image.empty()) {
      LOG(ERROR) << "invalid rgb image";
      return;
    }

    auto depth_img = cv_bridge::toCvCopy(depth_msg);

    cv::Mat depth;
    if (depth_img->image.type() == CV_32FC1) {
      depth = depth_img->image;
    } else if (depth_img->image.type() == CV_16UC1) {
      depth_img->image.convertTo(depth, CV_32FC1, 1.0e-3);
    } else {
      LOG(FATAL) << "Invalid depth type: " << depth_img->image.type();
      return;
    }

    const size_t total = rgb_msg->width * rgb_msg->height;
    voxblox::Pointcloud xyz(total);
    voxblox::Colors colors(total);
    fillPointCloud(rgb->image, depth, xyz, colors);

    const auto pose =
        buffer->lookupTransform(odom_frame, sensor_frame, rgb->header.stamp);

    ReconstructionInput input;
    tf2::convert(pose.transform.rotation, input.world_R_body);
    input.world_t_body << pose.transform.translation.x, pose.transform.translation.y,
        pose.transform.translation.z;

    FrameData data;
    CHECK(input.fillFrameData(data));
    tsdf_integrator->updateMap(*sensor, data, *map);

    VLOG(2) << "";
    VLOG(2) << "====================================================================";
    VLOG(2) << "Starting Incremental Update";
    VLOG(2) << "====================================================================";
    gvd_integrator->update(rgb_msg->header.stamp.toNSec(), *map, true, true);

    full_gvd.reset(new Layer<GvdVoxel>(map->voxel_size(), map->voxels_per_side()));
    full_gvd_integrator.reset(
        new ComboIntegrator(gvd_config, full_gvd, &mesh_config, graph_config));

    VLOG(2) << "";
    VLOG(2) << "====================================================================";
    VLOG(2) << "Running Full Update" << std::endl;
    VLOG(2) << "====================================================================";
    full_gvd_integrator->update(rgb_msg->header.stamp.toNSec(), *map, false, true);

    auto result = compareLayers(*gvd, *full_gvd);

    LOG(INFO) << std::endl
              << "**********************" << std::endl
              << "* Comparison Results *" << std::endl
              << "**********************" << std::endl
              << result;

    if (visualizer && gvd_integrator->graph_extractor) {
      const uint64_t timestamp_ns = rgb_msg->header.stamp.toNSec();
      visualizer->visualize(timestamp_ns, *gvd, gvd_integrator->graph_extractor.get());
      visualizer->visualizeError(timestamp_ns, *gvd, *full_gvd, 0.0);

      voxblox_msgs::Mesh mesh_msg;
      generateVoxbloxMeshMsg(map->getMeshLayer().getVoxbloxMesh(),
                             voxblox::ColorMode::kLambertColor,
                             &mesh_msg);
      mesh_msg.header.frame_id = odom_frame;
      mesh_msg.header.stamp = rgb_msg->header.stamp;
      mesh_pub.publish(mesh_msg);

      ros::spinOnce();
    }

    if (full_visualizer && full_gvd_integrator->graph_extractor) {
      const uint64_t timestamp_ns = rgb_msg->header.stamp.toNSec();
      full_visualizer->visualize(
          timestamp_ns, *full_gvd, full_gvd_integrator->graph_extractor.get());
      ros::spinOnce();
    }
  }

  void readBag(const std::string& bag_path, const BagConfig& config) {
    rosbag::Bag bag;
    LOG(INFO) << "Opening " << bag_path << " ...";
    bag.open(bag_path, rosbag::bagmode::Read);
    LOG(INFO) << "Opened " << bag_path;
    LOG(INFO) << "Adding bag tfs to buffer...";
    addTfsFromBag(bag);
    LOG(INFO) << "Added bag tfs to buffer";

    std::vector<std::string> topics{config.color_topic, config.depth_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    bool have_start = false;
    ros::Time start;

    TimeSync sync(Policy(10));
    sync.registerCallback(&GvdValidator::handleImages, this);

    for (const auto& m : view) {
      if (visualizer && !ros::ok()) {
        break;
      }

      const auto topic = m.getTopic();
      if (!have_start) {
        start = m.getTime();
        if (config.start >= 0.0) {
          start += ros::Duration(config.start);
        }
        have_start = true;
      }

      const auto diff_s = (m.getTime() - start).toSec();
      if (diff_s < 0.0) {
        VLOG(2) << "Skipping message " << std::abs(diff_s) << " [s] before start";
        continue;
      }

      if (config.duration >= 0.0 && diff_s > config.duration) {
        LOG(INFO) << "Reached end of duration: " << diff_s << " [s]";
        return;
      }

      auto msg = m.instantiate<sensor_msgs::Image>();
      if (!msg) {
        LOG(ERROR) << "unable to get image from " << topic;
        return;
      }

      if (topic == config.color_topic) {
        sync.add<0>(ros::MessageEvent<Image>(msg, m.getTime()));
      } else {
        sync.add<1>(ros::MessageEvent<Image>(msg, m.getTime()));
      }

      if (FLAGS_max_updates > 0 && num_updates >= FLAGS_max_updates) {
        break;
      }
    }

    bag.close();
  }

  GvdConfig gvd_config;
  MeshConfig mesh_config;
  ComboIntegrator::GraphExtractorConfig graph_config;
  double voxel_size;
  int voxels_per_side;
  std::string odom_frame = "odom";
  std::string sensor_frame = "left_cam";

  int64_t num_updates = 0;
  std::unique_ptr<tf2::BufferCore> buffer;

  std::unique_ptr<Sensor> sensor;
  std::unique_ptr<VolumetricMap> map;
  std::unique_ptr<ProjectiveIntegrator> tsdf_integrator;

  Layer<GvdVoxel>::Ptr gvd;
  std::unique_ptr<ComboIntegrator> gvd_integrator;
  std::unique_ptr<PlacesVisualizer> visualizer;

  Layer<GvdVoxel>::Ptr full_gvd;
  std::unique_ptr<ComboIntegrator> full_gvd_integrator;
  std::unique_ptr<PlacesVisualizer> full_visualizer;

  ros::Publisher mesh_pub;
};

}  // namespace hydra

int main(int argc, char** argv) {
  ros::init(argc, argv, "gvd_validator");
  ros::NodeHandle nh("");

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_v = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc <= 1) {
    LOG(FATAL) << "Missing required bag path";
    return 1;
  }

  const std::string bag_path(argv[1]);
  hydra::VolumetricMap::Config map_config;
  map_config.voxel_size = FLAGS_voxel_size;
  map_config.voxels_per_side = FLAGS_voxels_per_side;
  auto bag_config = config::fromYamlFile<hydra::BagConfig>(FLAGS_config);
  auto tsdf_config = config::fromYamlFile<hydra::TsdfConfig>(FLAGS_config);
  auto mesh_config = config::fromYamlFile<hydra::MeshConfig>(FLAGS_config);
  auto gvd_config = config::fromYamlFile<hydra::GvdConfig>(FLAGS_config);
  auto graph_config =
      config::fromYamlFile<hydra::ComboIntegrator::GraphExtractorConfig>(FLAGS_config);

  VLOG(1) << "BagConfig:" << std::endl << bag_config;
  VLOG(1) << "TsdfConfig:" << std::endl << tsdf_config;
  VLOG(1) << "MeshConfig:" << std::endl << mesh_config;
  VLOG(1) << "GvdConfig:" << std::endl << gvd_config;

  hydra::Camera::Config cam_config;
  cam_config.width = bag_config.image_width;
  cam_config.height = bag_config.image_height;
  cam_config.fx = bag_config.intrinsics.at(0);
  cam_config.fy = bag_config.intrinsics.at(1);
  cam_config.cx = bag_config.intrinsics.at(2);
  cam_config.cx = bag_config.intrinsics.at(3);
  // identity transform
  hydra::ParamSensorExtrinsics::Config extrinsics;
  cam_config.extrinsics =
      config::VirtualConfig<hydra::SensorExtrinsics>(extrinsics, "param");

  config::VirtualConfig<hydra::Sensor> sensor_config(cam_config, "camera");
  hydra::GvdValidator validator(
      sensor_config, map_config, tsdf_config, mesh_config, gvd_config, graph_config);
  validator.mesh_pub = nh.advertise<voxblox_msgs::Mesh>("mesh_viz", 1, true);
  validator.readBag(bag_path, bag_config);
  return 0;
}
