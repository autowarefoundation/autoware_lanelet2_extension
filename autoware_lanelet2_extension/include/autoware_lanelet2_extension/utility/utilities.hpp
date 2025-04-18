// Copyright 2015-2023 Autoware Foundation. All rights reserved.
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
// Authors: Kenji Miyake, Ryohsuke Mitsudome

#ifndef AUTOWARE_LANELET2_EXTENSION__UTILITY__UTILITIES_HPP_
#define AUTOWARE_LANELET2_EXTENSION__UTILITY__UTILITIES_HPP_

// NOLINTBEGIN(readability-identifier-naming)

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Forward.h>

#include <map>

namespace lanelet::utils
{

// @brief combine multiple lanelets into one, solely focusing on the shape and discarding any
// associated information such as ID and attributes. InvalId is set for the ID.
// @param lanelets to be combined.
lanelet::ConstLanelet combineLaneletsShape(const lanelet::ConstLanelets & lanelets);

lanelet::LineString3d generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const double resolution = 5.0);
lanelet::ConstLineString3d getCenterlineWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution = 5.0);
lanelet::ConstLineString3d getRightBoundWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution = 5.0);
lanelet::ConstLineString3d getLeftBoundWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution = 5.0);

lanelet::ConstLanelet getExpandedLanelet(
  const lanelet::ConstLanelet & lanelet_obj, const double left_offset, const double right_offset);

lanelet::ConstLanelets getExpandedLanelets(
  const lanelet::ConstLanelets & lanelet_obj, const double left_offset, const double right_offset);

/**
 * @brief  Apply a patch for centerline because the original implementation
 * doesn't have enough quality
 */
void overwriteLaneletsCenterline(
  lanelet::LaneletMapPtr lanelet_map, const double resolution = 5.0,
  const bool force_overwrite = false);

/**
 * @brief  Apply another patch for centerline because the overwriteLaneletsCenterline
 * has several limitations. See the following document in detail.
 * https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md#centerline
 * // NOLINT
 */
void overwriteLaneletsCenterlineWithWaypoints(
  lanelet::LaneletMapPtr lanelet_map, const double resolution = 5.0,
  const bool force_overwrite = false);

lanelet::ConstLanelets getConflictingLanelets(
  const lanelet::routing::RoutingGraphConstPtr & graph, const lanelet::ConstLanelet & lanelet);

bool lineStringWithWidthToPolygon(
  const lanelet::ConstLineString3d & linestring, lanelet::ConstPolygon3d * polygon);

bool lineStringToPolygon(
  const lanelet::ConstLineString3d & linestring, lanelet::ConstPolygon3d * polygon);

double getLaneletLength2d(const lanelet::ConstLanelet & lanelet);
double getLaneletLength3d(const lanelet::ConstLanelet & lanelet);
double getLaneletLength2d(const lanelet::ConstLanelets & lanelet_sequence);
double getLaneletLength3d(const lanelet::ConstLanelets & lanelet_sequence);

lanelet::ArcCoordinates getArcCoordinates(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose);

/**
 * @brief  This function uses the centerline for the ego to follow.
 * - when the `use_waypoints` in the autoware_map_loader is true,
 *   - the waypoints tag in the lanelet2::LaneletMapPtr is used instead of the centerline.
 * - when the `use_waypoints` in the autoware_map_loader is false,
 *   - the centerline in the lanelet2::LaneletMapPtr is used.
 */
lanelet::ArcCoordinates getArcCoordinatesOnEgoCenterline(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose,
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr);

lanelet::ConstLineString3d getClosestSegment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring);

lanelet::CompoundPolygon3d getPolygonFromArcLength(
  const lanelet::ConstLanelets & lanelets, const double s1, const double s2);
double getLaneletAngle(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point);
bool isInLanelet(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::ConstLanelet & lanelet,
  const double radius = 0.0);
geometry_msgs::msg::Pose getClosestCenterPose(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point);
double getLateralDistanceToCenterline(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose);
double getLateralDistanceToClosestLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose);
}  // namespace lanelet::utils

// NOLINTEND(readability-identifier-naming)

#endif  // AUTOWARE_LANELET2_EXTENSION__UTILITY__UTILITIES_HPP_
