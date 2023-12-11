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
#include <kimera_pgmo/utils/CommonFunctions.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <filesystem>

namespace hydra {

struct MeshPublisherNode {
  explicit MeshPublisherNode(const ros::NodeHandle& nh) : nh_(nh), mesh_frame_("map") {
    nh.getParam("mesh_frame", mesh_frame_);
    nh.getParam("mesh_filepath", mesh_filepath_);
    const auto mesh_path = std::filesystem::path(mesh_filepath_);
    if (!std::filesystem::exists(mesh_path)) {
      ROS_FATAL_STREAM("Invalid mesh path: " << mesh_filepath_);
      return;
    }

    valid_ = true;
    pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("mesh", 1, true);
    publishMesh();

    reload_service_ =
        nh_.advertiseService("reload", &MeshPublisherNode::handleReload, this);
  }

  ~MeshPublisherNode() = default;

  bool spin() const {
    if (!valid_) {
      return false;
    }

    ros::spin();
    return true;
  }

  bool handleReload(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    publishMesh();
    return true;
  }

  void publishMesh() {
    ROS_INFO_STREAM("Loading mesh from: " << mesh_filepath_);
    pcl::PolygonMesh mesh;
    kimera_pgmo::ReadMeshWithStampsFromPly(mesh_filepath_, mesh, nullptr);

    mesh_msgs::TriangleMeshStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = mesh_frame_;
    msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);
    pub_.publish(msg);
  }

  bool valid_ = false;
  ros::NodeHandle nh_;
  std::string mesh_frame_;

  ros::Publisher pub_;
  ros::ServiceServer reload_service_;
  std::string mesh_filepath_;
};

}  // namespace hydra

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mesh_publisher_node");

  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("~");
  hydra::MeshPublisherNode node(nh);
  if (!node.spin()) {
    return 1;
  }

  return 0;
}
