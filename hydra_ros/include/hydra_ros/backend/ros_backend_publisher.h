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
#include <hydra/backend/backend_config.h>
#include <hydra/common/module.h>
#include <hydra/common/robot_prefix_config.h>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace spark_dsg {
class ZmqSender;
}  // namespace spark_dsg

namespace hydra {

class RosBackendPublisher : public Module {
 public:
  RosBackendPublisher(const ros::NodeHandle& nh, const BackendConfig& config);

  virtual ~RosBackendPublisher();

  void start() override{};

  void stop() override{};

  void save(const LogSetup&) override{};

  void publish(const DynamicSceneGraph& graph,
               const kimera_pgmo::DeformationGraph& dgraph,
               size_t timestamp_ns);

 protected:
  virtual void publishPoseGraph(const DynamicSceneGraph& graph,
                                const kimera_pgmo::DeformationGraph& dgraph) const;

  virtual void publishDeformationGraphViz(const kimera_pgmo::DeformationGraph& dgraph,
                                          size_t timestamp_ns) const;

 protected:
  ros::NodeHandle nh_;
  BackendConfig config_;

  ros::Publisher mesh_mesh_edges_pub_;
  ros::Publisher pose_mesh_edges_pub_;
  ros::Publisher pose_graph_pub_;

  bool zmq_publish_mesh_;

  // Hack for temporary removal of label flickering
  size_t last_zmq_pub_time_;

  std::unique_ptr<spark_dsg::ZmqSender> zmq_sender_;
  std::unique_ptr<DsgSender> dsg_sender_;
};

}  // namespace hydra
