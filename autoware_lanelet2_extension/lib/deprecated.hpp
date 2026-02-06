// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
//
// Authors: Simon Thompson, Ryohsuke Mitsudome

// NOLINTBEGIN(readability-identifier-naming)

#ifndef AUTOWARE_LANELET2_EXTENSION__LIB__DEPRECATED_HPP_
#define AUTOWARE_LANELET2_EXTENSION__LIB__DEPRECATED_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace deprecated
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

lanelet::ConstLineString3d getClosestSegment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring);

double getLaneletAngle(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point);

lanelet::ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const lanelet::BasicPoint2d & search_point,
  const double range);

lanelet::ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Point & search_point,
  const double range);

lanelet::ConstLanelets getAllNeighborsRight(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet);

lanelet::ConstLanelets getAllNeighborsLeft(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet);

lanelet::ConstLanelets getAllNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet);

lanelet::ConstLanelets getLaneChangeableNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet);

bool getClosestLanelet(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose,
  lanelet::ConstLanelet * closest_lanelet_ptr);

}  // namespace deprecated
// NOLINTEND(readability-identifier-naming)

#endif  // AUTOWARE_LANELET2_EXTENSION__LIB__DEPRECATED_HPP_
