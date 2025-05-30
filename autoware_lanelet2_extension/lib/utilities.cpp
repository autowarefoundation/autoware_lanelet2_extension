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
// Authors: Kenji Miyake, Ryohsuke Mitsudome

// NOLINTBEGIN(readability-identifier-naming)

#include "autoware_lanelet2_extension/utility/utilities.hpp"

#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <iostream>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace lanelet::utils
{
namespace
{

/// @brief copy the z values between 2 containers based on the 2D arc lengths
/// @tparam T1 a container of 3D points
/// @tparam T2 a container of 3D points
/// @param from points from which the z values will be copied
/// @param to points to which the z values will be copied
template <typename T1, typename T2>
void copyZ(const T1 & from, T2 & to)
{
  if (from.empty() || to.empty()) return;
  to.front().z() = from.front().z();
  if (from.size() < 2 || to.size() < 2) return;
  to.back().z() = from.back().z();
  auto i_from = 1lu;
  auto s_from = lanelet::geometry::distance2d(from[0], from[1]);
  auto s_to = 0.0;
  auto s_from_prev = 0.0;
  for (auto i_to = 1lu; i_to + 1 < to.size(); ++i_to) {
    s_to += lanelet::geometry::distance2d(to[i_to - 1], to[i_to]);
    for (; s_from < s_to && i_from + 1 < from.size(); ++i_from) {
      s_from_prev = s_from;
      s_from += lanelet::geometry::distance2d(from[i_from], from[i_from + 1]);
    }
    const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
    to[i_to].z() = from[i_from - 1].z() + ratio * (from[i_from].z() - from[i_from - 1].z());
  }
}

[[maybe_unused]] bool exists(const std::vector<int> & array, const int element)
{
  return std::find(array.begin(), array.end(), element) != array.end();
}

/**
 * [getContactingLanelets retrieves id of lanelets which has distance 0m to
 * search_point]
 * @param  lanelet_map   [pointer to lanelet]
 * @param  trafficRules  [traffic rules to ignore lanelets that are not
 * traversable]
 * @param  search_point  [2D point used for searching]
 * @param  contacting_lanelet_ids [array of lanelet ids that is contacting with
 * search_point]
 */
[[maybe_unused]] void getContactingLanelets(
  const lanelet::LaneletMapPtr lanelet_map,
  const lanelet::traffic_rules::TrafficRulesPtr traffic_rules,
  const lanelet::BasicPoint2d & search_point, std::vector<int> * contacting_lanelet_ids)
{
  if (!lanelet_map) {
    std::cerr << "No lanelet map is set!" << std::endl;
    return;
  }

  if (contacting_lanelet_ids == nullptr) {
    std::cerr << __FUNCTION__ << " contacting_lanelet_ids is null pointer!" << std::endl;
    return;
  }

  for (const auto & ll : lanelet_map->laneletLayer) {
    if (!traffic_rules->canPass(ll)) {
      continue;
    }
    lanelet::BasicPolygon2d poly = ll.polygon2d().basicPolygon();
    double distance = lanelet::geometry::distance(poly, search_point);
    if (distance < std::numeric_limits<double>::epsilon()) {
      contacting_lanelet_ids->push_back(static_cast<int32_t>(ll.id()));
    }
  }
}

std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d & line_string)
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);

  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }

  return segment_distances;
}

std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));

  return accumulated_lengths;
}

std::pair<size_t, size_t> findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double target_length)
{
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (std::size_t i = 1; i < N; ++i) {
    if (
      accumulated_lengths.at(i - 1) <= target_length &&
      target_length <= accumulated_lengths.at(i)) {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error("No nearest point found.");
}

std::vector<lanelet::BasicPoint3d> resamplePoints(
  const lanelet::ConstLineString3d & line_string, const int num_segments)
{
  // Calculate length
  const auto line_length = static_cast<double>(lanelet::geometry::length(line_string));

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);
  if (accumulated_lengths.size() < 2) return {};

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const auto target_length = (static_cast<double>(i) / num_segments) * line_length;
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point =
      back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.emplace_back(target_point);
  }

  return resampled_points;
}
lanelet::LineString3d getLineStringFromArcLength(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::Points3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  if (start_index == 0) {
    return lanelet::LineString3d{lanelet::InvalId, points};
  }
  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = boost::geometry::distance(p1.basicPoint(), p2.basicPoint());
    if (accumulated_length + length > s1) {
      start_index = i;
      break;
    }
    accumulated_length += length;
  }
  if (start_index < linestring.size() - 1) {
    const auto & p1 = linestring[start_index];
    const auto & p2 = linestring[start_index + 1];
    const double residue = s1 - accumulated_length;
    const auto direction_vector = (p2.basicPoint() - p1.basicPoint()).normalized();
    const auto start_basic_point = p1.basicPoint() + residue * direction_vector;
    const auto start_point = lanelet::Point3d(lanelet::InvalId, start_basic_point);
    points.push_back(start_point);
  }

  accumulated_length = 0;
  size_t end_index = linestring.size();
  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = boost::geometry::distance(p1.basicPoint(), p2.basicPoint());
    if (accumulated_length + length > s2) {
      end_index = i;
      break;
    }
    accumulated_length += length;
  }

  for (size_t i = start_index + 1; i < end_index; i++) {
    const auto p = lanelet::Point3d(linestring[i]);
    points.push_back(p);
  }
  if (end_index < linestring.size() - 1) {
    const auto & p1 = linestring[end_index];
    const auto & p2 = linestring[end_index + 1];
    const double residue = s2 - accumulated_length;
    const auto direction_vector = (p2.basicPoint() - p1.basicPoint()).normalized();
    const auto end_basic_point = p1.basicPoint() + residue * direction_vector;
    const auto end_point = lanelet::Point3d(lanelet::InvalId, end_basic_point);
    points.push_back(end_point);
  }
  return lanelet::LineString3d{lanelet::InvalId, points};
}
}  // namespace

lanelet::ConstLanelet combineLaneletsShape(const lanelet::ConstLanelets & lanelets)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  lanelet::Points3d centers;
  for (const auto & llt : lanelets) {
    for (const auto & pt : llt.leftBound()) {
      lefts.push_back(lanelet::Point3d(pt));
    }
    for (const auto & pt : llt.rightBound()) {
      rights.push_back(lanelet::Point3d(pt));
    }
    for (const auto & pt : llt.centerline()) {
      centers.push_back(lanelet::Point3d(pt));
    }
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  const auto center_line = lanelet::LineString3d(lanelet::InvalId, centers);
  auto combined_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  combined_lanelet.setCenterline(center_line);
  return combined_lanelet;
}

lanelet::LineString3d generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const double resolution)
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2;
    const lanelet::Point3d center_point(
      lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
      center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

lanelet::ConstLineString3d getCenterlineWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution)
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2;

    const auto vec_right_2_left = (left_points.at(i) - right_points.at(i)).normalized();

    const auto offset_center_basic_point = center_basic_point + vec_right_2_left * offset;

    const lanelet::Point3d center_point(
      lanelet::utils::getId(), offset_center_basic_point.x(), offset_center_basic_point.y(),
      offset_center_basic_point.z());
    centerline.push_back(center_point);
  }
  return static_cast<lanelet::ConstLineString3d>(centerline);
}

lanelet::ConstLineString3d getRightBoundWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution)
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d rightBound(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++) {
    // Add ID for the average point of left and right
    const auto vec_left_2_right = (right_points.at(i) - left_points.at(i)).normalized();

    const auto offset_right_basic_point = right_points.at(i) + vec_left_2_right * offset;

    const lanelet::Point3d rightBound_point(
      lanelet::utils::getId(), offset_right_basic_point.x(), offset_right_basic_point.y(),
      offset_right_basic_point.z());
    rightBound.push_back(rightBound_point);
  }
  return static_cast<lanelet::ConstLineString3d>(rightBound);
}

lanelet::ConstLineString3d getLeftBoundWithOffset(
  const lanelet::ConstLanelet & lanelet_obj, const double offset, const double resolution)
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d leftBound(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++) {
    // Add ID for the average point of left and right

    const auto vec_right_2_left = (left_points.at(i) - right_points.at(i)).normalized();

    const auto offset_left_basic_point = left_points.at(i) + vec_right_2_left * offset;

    const lanelet::Point3d leftBound_point(
      lanelet::utils::getId(), offset_left_basic_point.x(), offset_left_basic_point.y(),
      offset_left_basic_point.z());
    leftBound.push_back(leftBound_point);
  }
  return static_cast<lanelet::ConstLineString3d>(leftBound);
}

lanelet::ConstLanelet getExpandedLanelet(
  const lanelet::ConstLanelet & lanelet_obj, const double left_offset, const double right_offset)
{
  using lanelet::geometry::offsetNoThrow;
  using lanelet::geometry::internal::checkForInversion;

  const auto & orig_left_bound_2d = lanelet_obj.leftBound2d().basicLineString();
  const auto & orig_right_bound_2d = lanelet_obj.rightBound2d().basicLineString();

  // Note: The lanelet::geometry::offset throws exception when the undesired inversion is found.
  // Use offsetNoThrow until the logic is updated to handle the inversion.
  // TODO(Horibe) update
  auto expanded_left_bound_2d = offsetNoThrow(orig_left_bound_2d, left_offset);
  auto expanded_right_bound_2d = offsetNoThrow(orig_right_bound_2d, right_offset);

  rclcpp::Clock clock{RCL_ROS_TIME};
  try {
    checkForInversion(orig_left_bound_2d, expanded_left_bound_2d, left_offset);
    checkForInversion(orig_right_bound_2d, expanded_right_bound_2d, right_offset);
  } catch (const lanelet::GeometryError & e) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("autoware_lanelet2_extension"), clock, 1000,
      "Fail to expand lanelet. output may be undesired. Lanelet points interval in map data could "
      "be too narrow.");
  }

  // Note: modify front and back points so that the successive lanelets will not have any
  // longitudinal space between them.
  {  // front
    const double diff_x = orig_right_bound_2d.front().x() - orig_left_bound_2d.front().x();
    const double diff_y = orig_right_bound_2d.front().y() - orig_left_bound_2d.front().y();
    const double theta = std::atan2(diff_y, diff_x);
    expanded_right_bound_2d.front().x() =
      orig_right_bound_2d.front().x() - right_offset * std::cos(theta);
    expanded_right_bound_2d.front().y() =
      orig_right_bound_2d.front().y() - right_offset * std::sin(theta);
    expanded_left_bound_2d.front().x() =
      orig_left_bound_2d.front().x() - left_offset * std::cos(theta);
    expanded_left_bound_2d.front().y() =
      orig_left_bound_2d.front().y() - left_offset * std::sin(theta);
  }
  {  // back
    const double diff_x = orig_right_bound_2d.back().x() - orig_left_bound_2d.back().x();
    const double diff_y = orig_right_bound_2d.back().y() - orig_left_bound_2d.back().y();
    const double theta = std::atan2(diff_y, diff_x);
    expanded_right_bound_2d.back().x() =
      orig_right_bound_2d.back().x() - right_offset * std::cos(theta);
    expanded_right_bound_2d.back().y() =
      orig_right_bound_2d.back().y() - right_offset * std::sin(theta);
    expanded_left_bound_2d.back().x() =
      orig_left_bound_2d.back().x() - left_offset * std::cos(theta);
    expanded_left_bound_2d.back().y() =
      orig_left_bound_2d.back().y() - left_offset * std::sin(theta);
  }

  const auto toPoints3d = [](const lanelet::BasicLineString2d & ls2d) {
    lanelet::Points3d output;
    for (const auto & pt : ls2d) {
      output.push_back(lanelet::Point3d(lanelet::InvalId, pt.x(), pt.y(), 0.0));
    }
    return output;
  };

  lanelet::Points3d ex_lefts = toPoints3d(expanded_left_bound_2d);
  lanelet::Points3d ex_rights = toPoints3d(expanded_right_bound_2d);
  copyZ(lanelet_obj.leftBound3d(), ex_lefts);
  copyZ(lanelet_obj.rightBound3d(), ex_rights);

  const auto & extended_left_bound_3d = lanelet::LineString3d(lanelet::InvalId, ex_lefts);
  const auto & expanded_right_bound_3d = lanelet::LineString3d(lanelet::InvalId, ex_rights);
  const auto & lanelet = lanelet::Lanelet(
    lanelet_obj.id(), extended_left_bound_3d, expanded_right_bound_3d, lanelet_obj.attributes());

  return lanelet;
}

lanelet::ConstLanelets getExpandedLanelets(
  const lanelet::ConstLanelets & lanelet_obj, const double left_offset, const double right_offset)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & llt : lanelet_obj) {
    lanelets.push_back(getExpandedLanelet(llt, left_offset, right_offset));
  }
  return lanelets;
}

void overwriteLaneletsCenterline(
  lanelet::LaneletMapPtr lanelet_map, const double resolution, const bool force_overwrite)
{
  for (auto & lanelet_obj : lanelet_map->laneletLayer) {
    if (force_overwrite || !lanelet_obj.hasCustomCenterline()) {
      const auto fine_center_line = generateFineCenterline(lanelet_obj, resolution);
      lanelet_obj.setCenterline(fine_center_line);
    }
  }
}

void overwriteLaneletsCenterlineWithWaypoints(
  lanelet::LaneletMapPtr lanelet_map, const double resolution, const bool force_overwrite)
{
  for (auto & lanelet_obj : lanelet_map->laneletLayer) {
    if (force_overwrite) {
      const auto fine_center_line = generateFineCenterline(lanelet_obj, resolution);
      lanelet_obj.setCenterline(fine_center_line);
    } else {
      if (lanelet_obj.hasCustomCenterline()) {
        const auto & centerline = lanelet_obj.centerline();
        lanelet_obj.setAttribute("waypoints", centerline.id());
      }

      const auto fine_center_line = generateFineCenterline(lanelet_obj, resolution);
      lanelet_obj.setCenterline(fine_center_line);
    }
  }
}

lanelet::ConstLanelets getConflictingLanelets(
  const lanelet::routing::RoutingGraphConstPtr & graph, const lanelet::ConstLanelet & lanelet)
{
  const auto & llt_or_areas = graph->conflicting(lanelet);
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(llt_or_areas.size());
  for (const auto & l_or_a : llt_or_areas) {
    auto llt_opt = l_or_a.lanelet();
    if (!!llt_opt) {
      lanelets.push_back(llt_opt.get());
    }
  }
  return lanelets;
}

bool lineStringWithWidthToPolygon(
  const lanelet::ConstLineString3d & linestring, lanelet::ConstPolygon3d * polygon)
{
  if (polygon == nullptr) {
    std::cerr << __func__ << ": polygon is null pointer! Failed to convert to polygon."
              << std::endl;
    return false;
  }
  if (linestring.size() != 2) {
    std::cerr << __func__ << ": linestring" << linestring.id() << " must have 2 points! ("
              << linestring.size() << " != 2)" << std::endl
              << "Failed to convert to polygon.";
    return false;
  }
  if (!linestring.hasAttribute("width")) {
    std::cerr << __func__ << ": linestring" << linestring.id()
              << " does not have width tag. Failed to convert to polygon.";
    return false;
  }

  const Eigen::Vector3d direction =
    linestring.back().basicPoint() - linestring.front().basicPoint();
  const double width = linestring.attributeOr("width", 0.0);
  const Eigen::Vector3d direction_left =
    (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) * direction).normalized();

  const Eigen::Vector3d eigen_p1 = linestring.front().basicPoint() + direction_left * width / 2;
  const Eigen::Vector3d eigen_p2 = linestring.back().basicPoint() + direction_left * width / 2;
  const Eigen::Vector3d eigen_p3 = linestring.back().basicPoint() - direction_left * width / 2;
  const Eigen::Vector3d eigen_p4 = linestring.front().basicPoint() - direction_left * width / 2;

  const lanelet::Point3d p1(lanelet::InvalId, eigen_p1.x(), eigen_p1.y(), eigen_p1.z());
  const lanelet::Point3d p2(lanelet::InvalId, eigen_p2.x(), eigen_p2.y(), eigen_p2.z());
  const lanelet::Point3d p3(lanelet::InvalId, eigen_p3.x(), eigen_p3.y(), eigen_p3.z());
  const lanelet::Point3d p4(lanelet::InvalId, eigen_p4.x(), eigen_p4.y(), eigen_p4.z());

  *polygon = lanelet::Polygon3d(lanelet::InvalId, {p1, p2, p3, p4});

  return true;
}

bool lineStringToPolygon(
  const lanelet::ConstLineString3d & linestring, lanelet::ConstPolygon3d * polygon)
{
  if (polygon == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("autoware_lanelet2_extension.visualization"),
      __func__ << ": polygon is null pointer! Failed to convert to polygon.");
    return false;
  }
  if (linestring.size() < 4) {
    if (linestring.size() < 3 || linestring.front().id() == linestring.back().id()) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("autoware_lanelet2_extension.visualization"),
        __func__ << ": linestring" << linestring.id()
                 << " must have more than different 3 points! (size is " << linestring.size()
                 << "). Failed to convert to polygon.");
      return false;
    }
  }

  lanelet::Polygon3d llt_poly;

  for (const auto & lp : linestring) {
    llt_poly.push_back(
      lanelet::Point3d(
        lanelet::InvalId, lp.basicPoint().x(), lp.basicPoint().y(), lp.basicPoint().z()));
  }

  if (linestring.front().id() == linestring.back().id()) {
    llt_poly.pop_back();
  }

  *polygon = llt_poly;

  return true;
}

double getLaneletLength2d(const lanelet::ConstLanelet & lanelet)
{
  return static_cast<double>(
    boost::geometry::length(lanelet::utils::to2D(lanelet.centerline()).basicLineString()));
}

double getLaneletLength3d(const lanelet::ConstLanelet & lanelet)
{
  return static_cast<double>(boost::geometry::length(lanelet.centerline().basicLineString()));
}

double getLaneletLength2d(const lanelet::ConstLanelets & lanelet_sequence)
{
  double length = 0;
  for (const auto & llt : lanelet_sequence) {
    length += getLaneletLength2d(llt);
  }
  return length;
}

double getLaneletLength3d(const lanelet::ConstLanelets & lanelet_sequence)
{
  double length = 0;
  for (const auto & llt : lanelet_sequence) {
    length += getLaneletLength3d(llt);
  }
  return length;
}

lanelet::ArcCoordinates getArcCoordinates(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose)
{
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lanelet);

  double length = 0;
  lanelet::ArcCoordinates arc_coordinates;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline_2d = lanelet::utils::to2D(llt.centerline());
    if (llt == closest_lanelet) {
      const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
      arc_coordinates = lanelet::geometry::toArcCoordinates(
        centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
      arc_coordinates.length += length;
      break;
    }
    length += static_cast<double>(boost::geometry::length(centerline_2d));
  }
  return arc_coordinates;
}

lanelet::ArcCoordinates getArcCoordinatesOnEgoCenterline(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose,
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lanelet);

  double length = 0;
  lanelet::ArcCoordinates arc_coordinates;
  for (const auto & llt : lanelet_sequence) {
    ConstLineString2d centerline_2d;
    if (llt.hasAttribute("waypoints")) {
      const auto waypoints_id = llt.attribute("waypoints").asId().value();
      centerline_2d = lanelet::utils::to2D(lanelet_map_ptr->lineStringLayer.get(waypoints_id));
    } else {
      centerline_2d = lanelet::utils::to2D(llt.centerline());
    }

    if (llt == closest_lanelet) {
      const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
      arc_coordinates = lanelet::geometry::toArcCoordinates(
        centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
      arc_coordinates.length += length;
      break;
    }
    length += static_cast<double>(boost::geometry::length(centerline_2d));
  }
  return arc_coordinates;
}

lanelet::ConstLineString3d getClosestSegment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring)
{
  if (linestring.size() < 2) {
    return lanelet::LineString3d();
  }

  lanelet::ConstLineString3d closest_segment;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 1; i < linestring.size(); i++) {
    lanelet::BasicPoint3d prev_basic_pt = linestring[i - 1].basicPoint();
    lanelet::BasicPoint3d current_basic_pt = linestring[i].basicPoint();

    lanelet::Point3d prev_pt(
      lanelet::InvalId, prev_basic_pt.x(), prev_basic_pt.y(), prev_basic_pt.z());
    lanelet::Point3d current_pt(
      lanelet::InvalId, current_basic_pt.x(), current_basic_pt.y(), current_basic_pt.z());

    lanelet::LineString3d current_segment(lanelet::InvalId, {prev_pt, current_pt});
    double distance = lanelet::geometry::distance2d(
      lanelet::utils::to2D(current_segment).basicLineString(), search_pt);
    if (distance < min_distance) {
      closest_segment = current_segment;
      min_distance = distance;
    }
  }
  return closest_segment;
}

lanelet::CompoundPolygon3d getPolygonFromArcLength(
  const lanelet::ConstLanelets & lanelets, const double s1, const double s2)
{
  const auto combined_lanelet = combineLaneletsShape(lanelets);
  const auto total_length = getLaneletLength2d(combined_lanelet);

  // make sure that s1, and s2 are between [0, lane_length]
  const auto s1_saturated = std::max(0.0, std::min(s1, total_length));
  const auto s2_saturated = std::max(0.0, std::min(s2, total_length));

  const auto ratio_s1 = s1_saturated / total_length;
  const auto ratio_s2 = s2_saturated / total_length;

  const auto s1_left = static_cast<double>(
    ratio_s1 * boost::geometry::length(combined_lanelet.leftBound().basicLineString()));
  const auto s2_left = static_cast<double>(
    ratio_s2 * boost::geometry::length(combined_lanelet.leftBound().basicLineString()));
  const auto s1_right = static_cast<double>(
    ratio_s1 * boost::geometry::length(combined_lanelet.rightBound().basicLineString()));
  const auto s2_right = static_cast<double>(
    ratio_s2 * boost::geometry::length(combined_lanelet.rightBound().basicLineString()));

  const auto left_bound =
    getLineStringFromArcLength(combined_lanelet.leftBound(), s1_left, s2_left);
  const auto right_bound =
    getLineStringFromArcLength(combined_lanelet.rightBound(), s1_right, s2_right);

  const auto & lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  return lanelet.polygon3d();
}

double getLaneletAngle(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point)
{
  lanelet::BasicPoint2d llt_search_point(search_point.x, search_point.y);
  lanelet::ConstLineString3d segment = getClosestSegment(llt_search_point, lanelet.centerline());
  return std::atan2(
    segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
}

bool isInLanelet(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::ConstLanelet & lanelet,
  const double radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(current_pose.position.x, current_pose.position.y);
  return boost::geometry::distance(p, lanelet.polygon2d().basicPolygon()) < radius + eps;
}

geometry_msgs::msg::Pose getClosestCenterPose(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point)
{
  lanelet::BasicPoint2d llt_search_point(search_point.x, search_point.y);

  if (lanelet.centerline().size() == 1) {
    geometry_msgs::msg::Pose closest_pose;
    closest_pose.position.x = lanelet.centerline().front().x();
    closest_pose.position.y = lanelet.centerline().front().y();
    closest_pose.position.z = search_point.z;
    closest_pose.orientation.x = 0.0;
    closest_pose.orientation.y = 0.0;
    closest_pose.orientation.z = 0.0;
    closest_pose.orientation.w = 1.0;
    return closest_pose;
  }

  lanelet::ConstLineString3d segment = getClosestSegment(llt_search_point, lanelet.centerline());
  if (segment.empty()) {
    return geometry_msgs::msg::Pose{};
  }

  const Eigen::Vector2d direction(
    (segment.back().basicPoint2d() - segment.front().basicPoint2d()).normalized());
  const Eigen::Vector2d xf(segment.front().basicPoint2d());
  const Eigen::Vector2d x(search_point.x, search_point.y);
  const Eigen::Vector2d p = xf + (x - xf).dot(direction) * direction;

  geometry_msgs::msg::Pose closest_pose;
  closest_pose.position.x = p.x();
  closest_pose.position.y = p.y();
  closest_pose.position.z = search_point.z;

  const double lane_yaw = getLaneletAngle(lanelet, search_point);
  tf2::Quaternion q;
  q.setRPY(0, 0, lane_yaw);
  closest_pose.orientation = tf2::toMsg(q);

  return closest_pose;
}

double getLateralDistanceToCenterline(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose)
{
  const auto & centerline_2d = lanelet::utils::to2D(lanelet.centerline());
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
  return lanelet::geometry::signedDistance(
    centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
}

double getLateralDistanceToClosestLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & pose)
{
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lanelet);
  return getLateralDistanceToCenterline(closest_lanelet, pose);
}
}  // namespace lanelet::utils

// NOLINTEND(readability-identifier-naming)
