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
#include "hydra_ros/utils/image_depth_adaptor.h"

#include <sensor_msgs/image_encodings.h>

namespace hydra {

template <typename T>
struct DepthParserImpl : DepthParser {
  T read_raw(const uint8_t* ptr) const {
    T value;
    std::memcpy(&value, ptr, sizeof(T));
    return value;
  }

  std::optional<float> read(const uint8_t* ptr) const override;

  size_t step() const override { return sizeof(T); }
};

template <>
std::optional<float> DepthParserImpl<float>::read(const uint8_t* ptr) const {
  const auto value = read_raw(ptr);
  if (std::isfinite(value)) {
    return value;
  } else {
    return std::nullopt;
  }
}

template <>
std::optional<float> DepthParserImpl<uint16_t>::read(const uint8_t* ptr) const {
  const auto value = read_raw(ptr);
  if (value != 0) {
    return 1.0e-3f * value;
  } else {
    return std::nullopt;
  }
}

DepthParser::Ptr createDepthParser(const std::string& encoding) {
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    return std::make_unique<DepthParserImpl<uint16_t>>();
  } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    return std::make_unique<DepthParserImpl<float>>();
  } else {
    return nullptr;
  }
}

DepthAdaptor::DepthAdaptor(const sensor_msgs::Image& img) {
  const auto channels = sensor_msgs::image_encodings::numChannels(img.encoding);
  if (channels != 1) {
    throw std::domain_error("invalid depth image encoding: " + img.encoding);
  }

  parser_ = createDepthParser(img.encoding);
}

std::optional<float> DepthAdaptor::read(const sensor_msgs::Image& img,
                                        uint32_t row,
                                        uint32_t col) const {
  const uint8_t* ptr = &img.data[0] + row * img.step + col * parser_->step();
  return parser_->read(ptr);
}

}  // namespace hydra
