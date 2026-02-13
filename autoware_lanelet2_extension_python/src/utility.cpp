// Copyright 2023 Autoware Foundation. All rights reserved.
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
// Authors: Mamoru Sobue

// NOLINTBEGIN(readability-identifier-naming)

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_python/internal/converter.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/utils.hpp>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bp = boost::python;

namespace impl
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

/**
 * utilities.cpp
 */

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

double getLaneletAngle(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & search_point)
{
  lanelet::BasicPoint2d llt_search_point(search_point.x, search_point.y);
  lanelet::ConstLineString3d segment = getClosestSegment(llt_search_point, lanelet.centerline());
  return std::atan2(
    segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
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

double getLaneletLength2d(const lanelet::ConstLanelets & lanelet_sequence)
{
  double length = 0;
  for (const auto & llt : lanelet_sequence) {
    length += lanelet::geometry::length2d(llt);
  }
  return length;
}

double getLaneletLength3d(const lanelet::ConstLanelets & lanelet_sequence)
{
  double length = 0;
  for (const auto & llt : lanelet_sequence) {
    length += lanelet::geometry::length3d(llt);
  }
  return length;
}

/**
 * query.cpp
 */

bool getClosestLanelet(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose,
  lanelet::ConstLanelet * closest_lanelet_ptr)
{
  if (closest_lanelet_ptr == nullptr) {
    std::cerr << "argument closest_lanelet_ptr is null! Failed to find closest lanelet"
              << std::endl;
    return false;
  }

  if (lanelets.empty()) {
    return false;
  }

  bool found = false;

  lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);

  // find by distance
  lanelet::ConstLanelets candidate_lanelets;
  {
    double min_distance = std::numeric_limits<double>::max();
    for (const auto & llt : lanelets) {
      double distance =
        boost::geometry::comparable_distance(llt.polygon2d().basicPolygon(), search_point);

      if (std::abs(distance - min_distance) <= std::numeric_limits<double>::epsilon()) {
        candidate_lanelets.push_back(llt);
      } else if (distance < min_distance) {
        found = true;
        candidate_lanelets.clear();
        candidate_lanelets.push_back(llt);
        min_distance = distance;
      }
    }
  }

  if (candidate_lanelets.size() == 1) {
    *closest_lanelet_ptr = candidate_lanelets.at(0);
    return found;
  }

  // find by angle
  {
    double min_angle = std::numeric_limits<double>::max();
    double pose_yaw = tf2::getYaw(search_pose.orientation);
    for (const auto & llt : candidate_lanelets) {
      lanelet::ConstLineString3d segment = getClosestSegment(search_point, llt.centerline());
      double angle_diff = M_PI;
      if (!segment.empty()) {
        double segment_angle = std::atan2(
          segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
        angle_diff = std::abs(normalize_radian(segment_angle - pose_yaw));
      }
      if (angle_diff < min_angle) {
        min_angle = angle_diff;
        *closest_lanelet_ptr = llt;
      }
    }
  }

  return found;
}

bool getClosestLaneletWithConstrains(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & search_pose,
  lanelet::ConstLanelet * closest_lanelet_ptr, const double dist_threshold,
  const double yaw_threshold)
{
  bool found = false;

  if (closest_lanelet_ptr == nullptr) {
    std::cerr << "argument closest_lanelet_ptr is null! Failed to find closest lanelet"
              << std::endl;
    return found;
  }

  if (lanelets.empty()) {
    return found;
  }

  lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);

  // find by distance
  std::vector<std::pair<lanelet::ConstLanelet, double>> candidate_lanelets;
  {
    for (const auto & llt : lanelets) {
      double distance = boost::geometry::distance(llt.polygon2d().basicPolygon(), search_point);

      if (distance <= dist_threshold) {
        candidate_lanelets.emplace_back(llt, distance);
      }
    }

    if (!candidate_lanelets.empty()) {
      // sort by distance
      std::sort(
        candidate_lanelets.begin(), candidate_lanelets.end(),
        [](
          const std::pair<lanelet::ConstLanelet, double> & x,
          std::pair<lanelet::ConstLanelet, double> & y) { return x.second < y.second; });
    } else {
      return found;
    }
  }

  // find closest lanelet within yaw_threshold
  {
    double min_angle = std::numeric_limits<double>::max();
    double min_distance = std::numeric_limits<double>::max();
    double pose_yaw = tf2::getYaw(search_pose.orientation);
    for (const auto & llt_pair : candidate_lanelets) {
      const auto & distance = llt_pair.second;

      double lanelet_angle = getLaneletAngle(llt_pair.first, search_pose.position);
      double angle_diff = std::abs(normalize_radian(lanelet_angle - pose_yaw));

      if (angle_diff > std::abs(yaw_threshold)) continue;
      if (min_distance < distance) break;

      if (angle_diff < min_angle) {
        min_angle = angle_diff;
        min_distance = distance;
        *closest_lanelet_ptr = llt_pair.first;
        found = true;
      }
    }
  }

  return found;
}

static lanelet::ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const lanelet::BasicPoint2d & search_point,
  const double range)
{
  lanelet::ConstLanelets near_lanelets;
  for (const auto & ll : lanelets) {
    lanelet::BasicPolygon2d poly = ll.polygon2d().basicPolygon();
    double distance = lanelet::geometry::distance(poly, search_point);
    if (distance <= range) {
      near_lanelets.push_back(ll);
    }
  }
  return near_lanelets;
}

static lanelet::ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Point & search_point,
  const double range)
{
  return getLaneletsWithinRange(
    lanelets, lanelet::BasicPoint2d(search_point.x, search_point.y), range);
}

static lanelet::ConstLanelets getAllNeighborsRight(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets lanelets;
  auto right_lane =
    (!!graph->right(lanelet)) ? graph->right(lanelet) : graph->adjacentRight(lanelet);
  while (!!right_lane) {
    lanelets.push_back(right_lane.get());
    right_lane = (!!graph->right(right_lane.get())) ? graph->right(right_lane.get())
                                                    : graph->adjacentRight(right_lane.get());
  }
  return lanelets;
}

static lanelet::ConstLanelets getAllNeighborsLeft(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets lanelets;
  auto left_lane = (!!graph->left(lanelet)) ? graph->left(lanelet) : graph->adjacentLeft(lanelet);
  while (!!left_lane) {
    lanelets.push_back(left_lane.get());
    left_lane = (!!graph->left(left_lane.get())) ? graph->left(left_lane.get())
                                                 : graph->adjacentLeft(left_lane.get());
  }
  return lanelets;
}

static lanelet::ConstLanelets getAllNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets lanelets;

  lanelet::ConstLanelets left_lanelets = getAllNeighborsLeft(graph, lanelet);
  lanelet::ConstLanelets right_lanelets = getAllNeighborsRight(graph, lanelet);

  std::reverse(left_lanelets.begin(), left_lanelets.end());
  lanelets.insert(lanelets.end(), left_lanelets.begin(), left_lanelets.end());
  lanelets.push_back(lanelet);
  lanelets.insert(lanelets.end(), right_lanelets.begin(), right_lanelets.end());

  return lanelets;
}

static lanelet::ConstLanelets getAllNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const geometry_msgs::msg::Point & search_point)
{
  const auto lanelets =
    getLaneletsWithinRange(road_lanelets, search_point, std::numeric_limits<double>::epsilon());
  lanelet::ConstLanelets road_slices;
  for (const auto & llt : lanelets) {
    const auto tmp_road_slice = getAllNeighbors(graph, llt);
    road_slices.insert(road_slices.end(), tmp_road_slice.begin(), tmp_road_slice.end());
  }
  return road_slices;
}

static lanelet::ConstLanelets getLaneChangeableNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet)
{
  return graph->besides(lanelet);
}

static lanelet::ConstLanelets getLaneChangeableNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const geometry_msgs::msg::Point & search_point)
{
  const auto lanelets =
    getLaneletsWithinRange(road_lanelets, search_point, std::numeric_limits<double>::epsilon());
  lanelet::ConstLanelets road_slices;
  for (const auto & llt : lanelets) {
    const auto tmp_road_slice = getLaneChangeableNeighbors(graph, llt);
    road_slices.insert(road_slices.end(), tmp_road_slice.begin(), tmp_road_slice.end());
  }
  return road_slices;
}

static std::vector<std::deque<lanelet::ConstLanelet>> getSucceedingLaneletSequencesRecursive(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet,
  const double length)
{
  std::vector<std::deque<lanelet::ConstLanelet>> succeeding_lanelet_sequences;

  const auto next_lanelets = graph->following(lanelet);
  const double lanelet_length = lanelet::geometry::length3d(lanelet);

  // end condition of the recursive function
  if (next_lanelets.empty() || lanelet_length >= length) {
    succeeding_lanelet_sequences.push_back({lanelet});
    return succeeding_lanelet_sequences;
  }

  for (const auto & next_lanelet : next_lanelets) {
    // get lanelet sequence after next_lanelet
    auto tmp_lanelet_sequences =
      getSucceedingLaneletSequencesRecursive(graph, next_lanelet, length - lanelet_length);
    for (auto & tmp_lanelet_sequence : tmp_lanelet_sequences) {
      tmp_lanelet_sequence.push_front(lanelet);
      succeeding_lanelet_sequences.push_back(tmp_lanelet_sequence);
    }
  }
  return succeeding_lanelet_sequences;
}

static std::vector<lanelet::ConstLanelets> getSucceedingLaneletSequences(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet,
  const double length)
{
  std::vector<lanelet::ConstLanelets> lanelet_sequences_vec;
  const auto next_lanelets = graph->following(lanelet);
  for (const auto & next_lanelet : next_lanelets) {
    const auto lanelet_sequences_deq =
      getSucceedingLaneletSequencesRecursive(graph, next_lanelet, length);
    for (const auto & lanelet_sequence : lanelet_sequences_deq) {
      lanelet_sequences_vec.emplace_back(lanelet_sequence.begin(), lanelet_sequence.end());
    }
  }
  return lanelet_sequences_vec;
}

static std::vector<std::deque<lanelet::ConstLanelet>> getPrecedingLaneletSequencesRecursive(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet,
  const double length, const lanelet::ConstLanelets & exclude_lanelets)
{
  std::vector<std::deque<lanelet::ConstLanelet>> preceding_lanelet_sequences;

  const auto prev_lanelets = graph->previous(lanelet);
  const double lanelet_length = lanelet::geometry::length3d(lanelet);

  // end condition of the recursive function
  if (prev_lanelets.empty() || lanelet_length >= length) {
    preceding_lanelet_sequences.push_back({lanelet});
    return preceding_lanelet_sequences;
  }

  for (const auto & prev_lanelet : prev_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, prev_lanelet)) {
      // if prev_lanelet is included in exclude_lanelets,
      // remove prev_lanelet from preceding_lanelet_sequences
      continue;
    }

    // get lanelet sequence after prev_lanelet
    auto tmp_lanelet_sequences = getPrecedingLaneletSequencesRecursive(
      graph, prev_lanelet, length - lanelet_length, exclude_lanelets);
    for (auto & tmp_lanelet_sequence : tmp_lanelet_sequences) {
      tmp_lanelet_sequence.push_back(lanelet);
      preceding_lanelet_sequences.push_back(tmp_lanelet_sequence);
    }
  }

  if (preceding_lanelet_sequences.empty()) {
    preceding_lanelet_sequences.push_back({lanelet});
  }
  return preceding_lanelet_sequences;
}

static std::vector<lanelet::ConstLanelets> getPrecedingLaneletSequences(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet,
  const double length, const lanelet::ConstLanelets & exclude_lanelets = {})
{
  std::vector<lanelet::ConstLanelets> lanelet_sequences_vec;
  const auto prev_lanelets = graph->previous(lanelet);
  for (const auto & prev_lanelet : prev_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, prev_lanelet)) {
      // if prev_lanelet is included in exclude_lanelets,
      // remove prev_lanelet from preceding_lanelet_sequences
      continue;
    }
    // convert deque into vector
    const auto lanelet_sequences_deq =
      getPrecedingLaneletSequencesRecursive(graph, prev_lanelet, length, exclude_lanelets);
    for (const auto & lanelet_sequence : lanelet_sequences_deq) {
      lanelet_sequences_vec.emplace_back(lanelet_sequence.begin(), lanelet_sequence.end());
    }
  }
  return lanelet_sequences_vec;
}

}  // namespace impl

namespace
{

/*
 * utilities.cpp
 */
lanelet::Optional<lanelet::ConstPolygon3d> lineStringWithWidthToPolygon(
  const lanelet::ConstLineString3d & linestring)
{
  lanelet::ConstPolygon3d poly{};
  if (lanelet::utils::lineStringWithWidthToPolygon(linestring, &poly)) {
    return poly;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> lineStringToPolygon(
  const lanelet::ConstLineString3d & linestring)
{
  lanelet::ConstPolygon3d poly{};
  if (lanelet::utils::lineStringToPolygon(linestring, &poly)) {
    return poly;
  }
  return {};
}

lanelet::ArcCoordinates getArcCoordinates(
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
}

double getLaneletAngle(const lanelet::ConstLanelet & lanelet, const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return ::impl::getLaneletAngle(lanelet, point);
}

bool isInLanelet(
  const std::string & pose_byte, const lanelet::ConstLanelet & lanelet, const double radius = 0.0)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::isInLanelet(pose, lanelet, radius);
}

std::vector<double> getClosestCenterPose(
  const lanelet::ConstLanelet & lanelet, const std::string & search_point_byte)
{
  rclcpp::SerializedMessage serialized_point_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_point_msg.reserve(message_header_length + search_point_byte.size());
  serialized_point_msg.get_rcl_serialized_message().buffer_length = search_point_byte.size();
  for (size_t i = 0; i < search_point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_point_msg.get_rcl_serialized_message().buffer[i] = search_point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point search_point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer_point;
  serializer_point.deserialize_message(&serialized_point_msg, &search_point);
  const geometry_msgs::msg::Pose pose = impl::getClosestCenterPose(lanelet, search_point);
  // NOTE: it was difficult to return the deserialized pose_byte and serialize the pose_byte on
  // python-side. So this function returns [*position, *quaternion] as double array
  const auto & xyz = pose.position;
  const auto & quat = pose.orientation;
  return std::vector<double>({xyz.x, xyz.y, xyz.z, quat.x, quat.y, quat.z, quat.w});
}

double getLateralDistanceToCenterline(
  const lanelet::ConstLanelet & lanelet, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getLateralDistanceToCenterline(lanelet, pose);
}

double getLateralDistanceToClosestLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getLateralDistanceToClosestLanelet(lanelet_sequence, pose);
}

/*
 * query.cpp
 */

lanelet::ConstLanelets subtypeLanelets(
  const lanelet::ConstLanelets & lls, const std::string & subtype)
{
  return lanelet::utils::query::subtypeLanelets(lls, subtype.c_str());
}

lanelet::Optional<lanelet::ConstLanelet> getLinkedLanelet(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstLanelets & all_road_lanelets,
  const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstLanelet linked_lanelet;
  if (lanelet::utils::query::getLinkedLanelet(
        parking_space, all_road_lanelets, all_parking_lots, &linked_lanelet)) {
    return linked_lanelet;
  }
  return {};
}

lanelet::Optional<lanelet::ConstLanelet> getLinkedLanelet(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelet linked_lanelet;
  if (lanelet::utils::query::getLinkedLanelet(parking_space, lanelet_map_ptr, &linked_lanelet)) {
    return linked_lanelet;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> getLinkedParkingLot(
  const lanelet::ConstLanelet & lanelet, const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstPolygon3d linked_parking_lot;
  if (lanelet::utils::query::getLinkedParkingLot(lanelet, all_parking_lots, &linked_parking_lot)) {
    return linked_parking_lot;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> getLinkedParkingLot(
  const lanelet::BasicPoint2d & current_position, const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstPolygon3d linked_parking_lot;
  if (lanelet::utils::query::getLinkedParkingLot(
        current_position, all_parking_lots, &linked_parking_lot)) {
    return linked_parking_lot;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> getLinkedParkingLot(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstPolygon3d linked_parking_lot;
  if (lanelet::utils::query::getLinkedParkingLot(
        parking_space, all_parking_lots, &linked_parking_lot)) {
    return linked_parking_lot;
  }
  return {};
}

lanelet::ConstLanelets getLaneletsWithinRange_point(
  const lanelet::ConstLanelets & lanelets, const std::string & point_byte, const double range)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return impl::getLaneletsWithinRange(lanelets, point, range);
}

lanelet::ConstLanelets getLaneChangeableNeighbors_point(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return impl::getLaneChangeableNeighbors(graph, road_lanelets, point);
}

lanelet::ConstLanelets getAllNeighbors_point(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return impl::getAllNeighbors(graph, road_lanelets, point);
}

lanelet::Optional<lanelet::ConstLanelet> getClosestLanelet(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  lanelet::ConstLanelet closest_lanelet{};
  if (impl::getClosestLanelet(lanelets, pose, &closest_lanelet)) {
    return closest_lanelet;
  }
  return {};
}

lanelet::Optional<lanelet::ConstLanelet> getClosestLaneletWithConstrains(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  lanelet::ConstLanelet closest_lanelet{};
  if (impl::getClosestLaneletWithConstrains(
        lanelets, pose, &closest_lanelet, dist_threshold, yaw_threshold)) {
    return closest_lanelet;
  }
  return {};
}

lanelet::ConstLanelets getCurrentLanelets_point(
  const lanelet::ConstLanelets & lanelets, const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  lanelet::ConstLanelets current_lanelets{};
  lanelet::utils::query::getCurrentLanelets(lanelets, point, &current_lanelets);
  return current_lanelets;
}

lanelet::ConstLanelets getCurrentLanelets_pose(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  lanelet::ConstLanelets current_lanelets{};
  lanelet::utils::query::getCurrentLanelets(lanelets, pose, &current_lanelets);
  return current_lanelets;
}

}  // namespace

// for handling functions with default arguments
/// utilities.cpp
// NOLINTBEGIN(google-explicit-constructor)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  generateFineCenterline_overload, lanelet::utils::generateFineCenterline, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getCenterlineWithOffset_overload, lanelet::utils::getCenterlineWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getRightBoundWithOffset_overload, lanelet::utils::getRightBoundWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getLeftBoundWithOffset_overload, lanelet::utils::getLeftBoundWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  overwriteLaneletsCenterline_overload, lanelet::utils::overwriteLaneletsCenterline, 1, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(isInLanelet_overload, ::isInLanelet, 2, 3)

/// query.cpp
BOOST_PYTHON_FUNCTION_OVERLOADS(
  stopSignStopLines_overload, lanelet::utils::query::stopSignStopLines, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getClosestLaneletWithConstrains_overload, ::getClosestLaneletWithConstrains, 2, 4)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getPrecedingLaneletSequences_overload, impl::getPrecedingLaneletSequences, 3, 4)
// NOLINTEND(google-explicit-constructor)

BOOST_PYTHON_MODULE(_autoware_lanelet2_extension_python_boost_python_utility)
{
  /*
   * utilities.cpp
   */
  bp::def("combineLaneletsShape", lanelet::utils::combineLaneletsShape);
  bp::def(
    "generateFineCenterline", lanelet::utils::generateFineCenterline,
    generateFineCenterline_overload());
  bp::def(
    "getCenterlineWithOffset", lanelet::utils::getCenterlineWithOffset,
    getCenterlineWithOffset_overload());
  bp::def(
    "getRightBoundWithOffset", lanelet::utils::getRightBoundWithOffset,
    getRightBoundWithOffset_overload());
  bp::def(
    "getLeftBoundWithOffset", lanelet::utils::getLeftBoundWithOffset,
    getLeftBoundWithOffset_overload());
  bp::def("getExpandedLanelet", lanelet::utils::getExpandedLanelet);
  bp::def("getExpandedLanelets", lanelet::utils::getExpandedLanelets);
  bp::def(
    "overwriteLaneletsCenterline", lanelet::utils::overwriteLaneletsCenterline,
    overwriteLaneletsCenterline_overload());
  bp::def("getConflictingLanelets", impl::getConflictingLanelets);
  bp::def("lineStringWithWidthToPolygon", ::lineStringWithWidthToPolygon);
  bp::def("lineStringToPolygon", ::lineStringToPolygon);
  bp::def<double(const lanelet::ConstLanelet &)>("getLaneletLength2d", lanelet::geometry::length2d);
  bp::def<double(const lanelet::ConstLanelet &)>("getLaneletLength3d", lanelet::geometry::length3d);
  bp::def<double(const lanelet::ConstLanelets &)>("getLaneletLength2d", impl::getLaneletLength2d);
  bp::def<double(const lanelet::ConstLanelets &)>("getLaneletLength3d", impl::getLaneletLength3d);
  bp::def("getArcCoordinates", ::getArcCoordinates);  // depends ros msg
  bp::def("getClosestSegment", impl::getClosestSegment);
  bp::def("getPolygonFromArcLength", lanelet::utils::getPolygonFromArcLength);
  bp::def("getLaneletAngle", ::getLaneletAngle);                  // depends on ros msg
  bp::def("isInLanelet", ::isInLanelet, isInLanelet_overload());  // depends ros msg
  bp::def("getClosestCenterPose", ::getClosestCenterPose);        // depends ros msg
  // NOTE: required for the return-value of getClosestCenterPose
  bp::class_<std::vector<double>>("[position, quaternion]")
    .def(bp::vector_indexing_suite<std::vector<double>>());
  bp::def("getLateralDistanceToCenterline", ::getLateralDistanceToCenterline);  // depends ros msg
  bp::def(
    "getLateralDistanceToClosestLanelet", ::getLateralDistanceToClosestLanelet);  // depends ros msg

  /*
   * query.cpp
   */
  bp::def("laneletLayer", lanelet::utils::query::laneletLayer);
  bp::def("subtypeLanelets", ::subtypeLanelets);
  bp::def("crosswalkLanelets", lanelet::utils::query::crosswalkLanelets);
  bp::def("walkwayLanelets", lanelet::utils::query::walkwayLanelets);
  bp::def("roadLanelets", lanelet::utils::query::roadLanelets);
  bp::def("shoulderLanelets", lanelet::utils::query::shoulderLanelets);
  bp::def("trafficLights", lanelet::utils::query::trafficLights);

  bp::def("autowareTrafficLights", lanelet::utils::query::autowareTrafficLights);
  converters::VectorToListConverter<std::vector<lanelet::AutowareTrafficLightConstPtr>>();

  bp::def("detectionAreas", lanelet::utils::query::detectionAreas);
  converters::VectorToListConverter<std::vector<lanelet::DetectionAreaConstPtr>>();

  bp::def("noStoppingAreas", lanelet::utils::query::noStoppingAreas);
  converters::VectorToListConverter<std::vector<lanelet::NoStoppingAreaConstPtr>>();

  bp::def("noParkingAreas", lanelet::utils::query::noParkingAreas);
  converters::VectorToListConverter<std::vector<lanelet::NoParkingAreaConstPtr>>();

  bp::def("speedBumps", lanelet::utils::query::speedBumps);
  converters::VectorToListConverter<std::vector<lanelet::SpeedBumpConstPtr>>();

  bp::def("crosswalks", lanelet::utils::query::crosswalks);
  converters::VectorToListConverter<std::vector<lanelet::CrosswalkConstPtr>>();

  bp::def("curbstones", lanelet::utils::query::curbstones);
  bp::def("getAllPolygonsByType", lanelet::utils::query::getAllPolygonsByType);
  bp::def("getAllObstaclePolygons", lanelet::utils::query::getAllObstaclePolygons);
  bp::def("getAllParkingLots", lanelet::utils::query::getAllParkingLots);
  bp::def("getAllPartitions", lanelet::utils::query::getAllPartitions);
  bp::def("getAllFences", lanelet::utils::query::getAllFences);
  bp::def(
    "getAllPedestrianPolygonMarkings", lanelet::utils::query::getAllPedestrianPolygonMarkings);
  bp::def("getAllPedestrianLineMarkings", lanelet::utils::query::getAllPedestrianLineMarkings);
  bp::def("getAllParkingSpaces", lanelet::utils::query::getAllParkingSpaces);

  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstLanelet &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstLanelet &, const lanelet::ConstLineStrings3d &,
    const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  // NOTE: required for iterating the return-value of getLinkedParkingSpaces/getAllParkingLots, but
  // this causes RuntimeWarning for duplicate to-Python converter
  // bp::class_<lanelet::ConstLineStrings3d>("lanelet::ConstLineStrings3d")
  //  .def(bp::vector_indexing_suite<lanelet::ConstLineStrings3d>());
  // bp::class_<lanelet::ConstPolygons3d>("lanelet::ConstPolygons3d")
  //  .def(bp::vector_indexing_suite<lanelet::ConstPolygons3d>());

  bp::def<lanelet::Optional<lanelet::ConstLanelet>(
    const lanelet::ConstLineString3d &, const lanelet::ConstLanelets &,
    const lanelet::ConstPolygons3d &)>("getLinkedLanelet", ::getLinkedLanelet);
  bp::def<lanelet::Optional<lanelet::ConstLanelet>(
    const lanelet::ConstLineString3d &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedLanelet", ::getLinkedLanelet);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLineString3d &, const lanelet::ConstLanelets &,
    const lanelet::ConstPolygons3d &)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLineString3d &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def<lanelet::Optional<lanelet::ConstPolygon3d>(
    const lanelet::ConstLanelet &, const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingLot", ::getLinkedParkingLot);
  bp::def<lanelet::Optional<lanelet::ConstPolygon3d>(
    const lanelet::BasicPoint2d &, const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingLot", ::getLinkedParkingLot);
  bp::def<lanelet::Optional<lanelet::ConstPolygon3d>(
    const lanelet::ConstLineString3d &, const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingLot", ::getLinkedParkingLot);
  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstPolygon3d & parking_lot,
    const lanelet::ConstLineStrings3d & all_parking_spaces)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstPolygon3d & parking_lot, const lanelet::ConstLanelets & all_road_lanelets)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def("stopLinesLanelets", lanelet::utils::query::stopLinesLanelets);
  bp::def("stopLinesLanelet", lanelet::utils::query::stopLinesLanelet);
  bp::def(
    "stopSignStopLines", lanelet::utils::query::stopSignStopLines, stopSignStopLines_overload());
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLanelets &, const lanelet::BasicPoint2d &, const double)>(
    "getLaneletsWithinRange", impl::getLaneletsWithinRange);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLanelets &, const std::string &, const double)>(
    "getLaneletsWithinRange_point", ::getLaneletsWithinRange_point);  // depends on ros msg
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelet &)>(
    "getLaneChangeableNeighbors", impl::getLaneChangeableNeighbors);
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelets &,
    const std::string &)>(
    "getLaneChangeableNeighbors_point", ::getLaneChangeableNeighbors_point);  // depends on ros msg
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelet &)>(
    "getAllNeighbors", impl::getAllNeighbors);
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelets &,
    const std::string &)>("getAllNeighbors_point", ::getAllNeighbors_point);  // depends on ros msg
  bp::def("getAllNeighborsLeft", impl::getAllNeighborsLeft);
  bp::def("getAllNeighborsRight", impl::getAllNeighborsRight);
  bp::def("getClosestLanelet", ::getClosestLanelet);  // depends on ros msg
  bp::def(
    "getClosestLaneletWithConstrains", ::getClosestLaneletWithConstrains,
    getClosestLaneletWithConstrains_overload());                    // depends on ros msg
  bp::def("getCurrentLanelets_point", ::getCurrentLanelets_point);  // depends on ros msg
  bp::def("getCurrentLanelets_pose", ::getCurrentLanelets_pose);    // depends on ros msg
  // NOTE: this is required for iterating getCurrentLanelets return value directly
  // bp::class_<lanelet::ConstLanelets>("lanelet::ConstLanelets")
  //  .def(bp::vector_indexing_suite<lanelet::ConstLanelets>());
  // NOTE: this is required for return-type of getSucceeding/PrecedingLaneletSequences
  // bp::class_<std::vector<lanelet::ConstLanelets>>("std::vector<lanelet::ConstLanelets>")
  //  .def(bp::vector_indexing_suite<std::vector<lanelet::ConstLanelets>>());
  bp::def("getSucceedingLaneletSequences", impl::getSucceedingLaneletSequences);
  bp::def(
    "getPrecedingLaneletSequences", impl::getPrecedingLaneletSequences,
    getPrecedingLaneletSequences_overload());
}

// NOLINTEND(readability-identifier-naming)
