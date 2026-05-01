// Copyright 2022 TIER IV, Inc.
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

// NOLINTBEGIN(readability-identifier-naming)

#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/route_checker.hpp"

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>

#include <memory>

using lanelet::Area;
using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::LineStrings3d;
using lanelet::Point3d;
using lanelet::utils::getId;

class TestSuite : public ::testing::Test  // NOLINT for gtest
{
public:
  TestSuite() : sample_map_ptr(new lanelet::LaneletMap())
  {
    // create sample lanelets
    const Point3d p1(getId(), 0.0, 0.0, 0.0);
    const Point3d p2(getId(), 0.0, 1.0, 0.0);

    const LineString3d ls_left(getId(), {p1, p2});

    const Point3d p3(getId(), 1.0, 0.0, 0.0);
    const Point3d p4(getId(), 1.0, 0.0, 0.0);

    const LineString3d ls_right(getId(), {p3, p4});

    const Lanelet lanelet(getId(), ls_left, ls_right);

    sample_map_ptr->add(lanelet);

    // create sample routes
    autoware_planning_msgs::msg::LaneletPrimitive map_primitive;
    autoware_planning_msgs::msg::LaneletSegment map_segment1;
    autoware_planning_msgs::msg::LaneletSegment map_segment2;

    for (size_t i = 0; i < 2; i++) {
      map_primitive.id = lanelet.id();
      map_segment1.primitives.push_back(map_primitive);
      map_primitive.id = ls_left.id();
      map_segment2.primitives.push_back(map_primitive);
    }
    sample_route1.segments.push_back(map_segment1);
    sample_route2.segments.push_back(map_segment2);
  }

  ~TestSuite() override = default;

  lanelet::LaneletMapPtr sample_map_ptr;
  autoware_planning_msgs::msg::LaneletRoute sample_route1;  // valid route
  autoware_planning_msgs::msg::LaneletRoute sample_route2;  // invalid route

private:
};

TEST_F(TestSuite, isRouteValid)  // NOLINT for gtest
{
  const auto route_ptr1 =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(sample_route1);
  const auto route_ptr2 =
    std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(sample_route2);

  ASSERT_TRUE(lanelet::utils::route::isRouteValid(*route_ptr1, sample_map_ptr))
    << "The route should be valid, which should be created on the same map as the current one";
  ASSERT_FALSE(lanelet::utils::route::isRouteValid(*route_ptr2, sample_map_ptr))
    << "The route should be invalid, which should be created on the different map from the current "
       "one";
}

TEST_F(TestSuite, isRouteValid_rejects_area_when_allow_area_false)  // NOLINT for gtest
{
  autoware_planning_msgs::msg::LaneletRoute route_with_area;
  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletPrimitive prim;
  prim.primitive_type = "area";
  prim.id = 99999;  // id is irrelevant when allow_area is false
  segment.primitives.push_back(prim);
  route_with_area.segments.push_back(segment);

  ASSERT_FALSE(lanelet::utils::route::isRouteValid(route_with_area, sample_map_ptr, false))
    << "Route with area primitive must be invalid when allow_area is false";
  ASSERT_FALSE(lanelet::utils::route::isRouteValid(route_with_area, sample_map_ptr))
    << "Default allow_area should be false for mixed routes";
}

TEST_F(TestSuite, isRouteValid_unknown_area_id_fails_when_allow_area_true)  // NOLINT for gtest
{
  autoware_planning_msgs::msg::LaneletRoute route_with_area;
  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletPrimitive prim;
  prim.primitive_type = "area";
  prim.id = 99999;  // not on map
  segment.primitives.push_back(prim);
  route_with_area.segments.push_back(segment);

  ASSERT_FALSE(lanelet::utils::route::isRouteValid(route_with_area, sample_map_ptr, true))
    << "Missing area id on map should fail validation";
}

TEST_F(TestSuite, isRouteValid_resolves_area_when_allow_area_true)  // NOLINT for gtest
{
  const lanelet::Id area_id = getId();
  const Point3d a1(getId(), 10.0, 0.0, 0.0);
  const Point3d a2(getId(), 12.0, 0.0, 0.0);
  const Point3d a3(getId(), 12.0, 2.0, 0.0);
  const Point3d a4(getId(), 10.0, 2.0, 0.0);
  const LineString3d ring(getId(), {a1, a2, a3, a4, a1});
  const LineStrings3d outer{ring};
  const Area test_area(area_id, outer);
  sample_map_ptr->add(test_area);

  autoware_planning_msgs::msg::LaneletRoute route_with_area;
  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletPrimitive prim;
  prim.primitive_type = "area";
  prim.id = area_id;
  segment.primitives.push_back(prim);
  route_with_area.segments.push_back(segment);

  ASSERT_TRUE(lanelet::utils::route::isRouteValid(route_with_area, sample_map_ptr, true))
    << "Valid area id on map should pass when allow_area is true";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// NOLINTEND(readability-identifier-naming)
