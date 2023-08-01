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
#include "hydra_ros/utils/image_label_adaptor.h"

#include <hydra/common/hydra_config.h>
#include <kimera_semantics/color.h>
#include <sensor_msgs/image_encodings.h>

#include "hydra_ros/utils/image_color_adaptor.h"

namespace hydra {
template <typename T>
struct LabelParserImpl : LabelParser {
  uint32_t read(const uint8_t* ptr) const override {
    T value;
    std::memcpy(&value, ptr, sizeof(T));
    return static_cast<uint32_t>(value);
  }

  size_t step() const override { return sizeof(T); }
};

struct ColorLabelParser : LabelParser {
  ColorLabelParser(const std::string& encoding)
      : color_parser(createColorParser(encoding)) {
    label_map = HydraConfig::instance().getSemanticColorMap();
    if (!label_map || !label_map->isValid()) {
      throw std::runtime_error("colormap required for color label image conversion");
    }
  }

  uint32_t read(const uint8_t* ptr) const override {
    const auto rgba = color_parser.read(ptr);
    return label_map->getLabelFromColor(
        voxblox::Color(rgba[0], rgba[1], rgba[2], rgba[3]));
  }

  size_t step() const override { return color_parser.step(); }

  const ColorParser color_parser;
  std::shared_ptr<kimera::SemanticColorMap> label_map;
};

LabelParser::Ptr createLabelParser(const std::string& encoding) {
  const auto depth = sensor_msgs::image_encodings::bitDepth(encoding);
  if (depth == 8) {
    return std::make_unique<LabelParserImpl<uint8_t>>();
  } else if (depth == 16) {
    return std::make_unique<LabelParserImpl<uint16_t>>();
  } else if (depth == 32) {
    return std::make_unique<LabelParserImpl<uint32_t>>();
  } else {
    return nullptr;
  }
}

LabelAdaptor::LabelAdaptor(const sensor_msgs::Image& img) {
  const auto channels = sensor_msgs::image_encodings::numChannels(img.encoding);
  if (channels == 1) {
    parser_ = createLabelParser(img.encoding);
    return;
  }

  if (channels != 3 && channels == 4) {
    throw std::domain_error("invalid label image encoding: " + img.encoding);
  }

  parser_.reset(new ColorLabelParser(img.encoding));
}

uint32_t LabelAdaptor::read(const sensor_msgs::Image& img,
                            uint32_t row,
                            uint32_t col) const {
  const uint8_t* ptr = &img.data[0] + row * img.step + col * parser_->step();
  return parser_->read(ptr);
}

}  // namespace hydra
