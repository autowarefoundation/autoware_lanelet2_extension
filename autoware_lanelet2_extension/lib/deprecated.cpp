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

// NOLINTEND(readability-identifier-naming)

#include "deprecated.hpp"

#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <iostream>
#include <limits>

using lanelet::utils::to2D;

namespace deprecated
{

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

lanelet::ConstLanelets getLaneletsWithinRange(
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

lanelet::ConstLanelets getLaneletsWithinRange(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Point & search_point,
  const double range)
{
  return getLaneletsWithinRange(
    lanelets, lanelet::BasicPoint2d(search_point.x, search_point.y), range);
}

lanelet::ConstLanelets getAllNeighborsRight(
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

lanelet::ConstLanelets getAllNeighborsLeft(
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

lanelet::ConstLanelets getAllNeighbors(
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

lanelet::ConstLanelets getLaneChangeableNeighbors(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet)
{
  return graph->besides(lanelet);
}

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

}  // namespace deprecated
