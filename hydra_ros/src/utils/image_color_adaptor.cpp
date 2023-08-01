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
#include "hydra_ros/utils/image_color_adaptor.h"

#include <sensor_msgs/image_encodings.h>

namespace hydra {

ColorParser::ColorParser(size_t red_offset,
                         size_t green_offset,
                         size_t blue_offset,
                         size_t pixel_step,
                         int64_t alpha_offset)
    : red_offset(red_offset),
      green_offset(green_offset),
      blue_offset(blue_offset),
      pixel_step(pixel_step),
      alpha_offset(alpha_offset) {}

std::array<uint8_t, 4> ColorParser::read(const uint8_t* ptr) const {
  std::array<uint8_t, 4> color;
  color[0] = *(ptr + red_offset);
  color[1] = *(ptr + green_offset);
  color[2] = *(ptr + blue_offset);
  color[3] = (alpha_offset == -1) ? 255 : *(ptr + alpha_offset);
  return color;
}

size_t ColorParser::step() const { return pixel_step; }

ColorParser createColorParser(const std::string& encoding) {
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return ColorParser(0, 1, 2, 3);
  } else if (encoding == sensor_msgs::image_encodings::BGR8) {
    return ColorParser(2, 1, 0, 3);
  } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
    return ColorParser(0, 1, 2, 4, 3);
  } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
    return ColorParser(2, 1, 0, 4, 3);
  } else if (encoding == sensor_msgs::image_encodings::MONO8) {
    return ColorParser(0, 0, 0, 1);
  } else {
    throw std::domain_error("invalid color image encoding: " + encoding);
  }
}

ColorAdaptor::ColorAdaptor(const sensor_msgs::Image& color)
    : parser(createColorParser(color.encoding)) {}

std::array<uint8_t, 4> ColorAdaptor::read(const sensor_msgs::Image& img,
                                          uint32_t row,
                                          uint32_t col) const {
  const uint8_t* rgb = &img.data[0] + row * img.step + col * parser.step();
  return parser.read(rgb);
}

}  // namespace hydra
