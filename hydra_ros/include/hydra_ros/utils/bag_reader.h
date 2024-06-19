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
#include <hydra/common/output_sink.h>
#include <sensor_msgs/Image.h>
#include <hydra/input/input_data.h>

#include <filesystem>

namespace hydra {

struct BagConfig {
  std::filesystem::path bag_path;
  std::string color_topic;
  std::string depth_topic;
  double start = -1.0;
  double duration = -1.0;
  bool color_compressed = false;
  config::VirtualConfig<Sensor> sensor;
  std::string sensor_frame;
  std::string world_frame;
};

void declare_config(BagConfig& config);

class PoseCache;

class BagReader {
 public:
  using Sink = OutputSink<const InputData&>;
  struct Config {
    std::vector<BagConfig> bags;
    std::vector<Sink::Factory> sinks;
  } const config;

  explicit BagReader(const Config& config);

  ~BagReader() = default;

  void read();

  void addSink(const Sink::Ptr& sink);

  void handleImages(const BagConfig& bag_config,
                    const Sensor::ConstPtr& sensor,
                    const PoseCache& cache,
                    const sensor_msgs::Image::ConstPtr& color_msg,
                    const sensor_msgs::Image::ConstPtr& depth_msg);

 protected:
  void readBag(const BagConfig& config);

  Sink::List sinks_;
};

void declare_config(BagReader::Config& config);

}  // namespace hydra
