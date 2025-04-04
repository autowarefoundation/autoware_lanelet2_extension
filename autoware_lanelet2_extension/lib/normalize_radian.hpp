// Copyright 2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <utility>

#ifndef AUTOWARE_LANELET2_EXTENSION__LIB__NORMALIZE_RADIAN_HPP_
#define AUTOWARE_LANELET2_EXTENSION__LIB__NORMALIZE_RADIAN_HPP_

namespace lanelet::utils::impl
{
inline double normalize_radian(const double rad)
{
  constexpr double pi = 3.14159265358979323846;  // To be replaced by std::numbers::pi in C++20
  constexpr double min_rad = -pi;
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}
}  // namespace lanelet::utils::impl

#endif  // AUTOWARE_LANELET2_EXTENSION__LIB__NORMALIZE_RADIAN_HPP_
