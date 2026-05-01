// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE_LANELET2_EXTENSION__TRAFFIC_RULES__AUTOWARE_TRAFFIC_RULES_HPP_
#define AUTOWARE_LANELET2_EXTENSION__TRAFFIC_RULES__AUTOWARE_TRAFFIC_RULES_HPP_

#include <lanelet2_traffic_rules/GermanTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace lanelet::autoware
{

/// @brief Autoware-specific location identifier for traffic rules registration.
static constexpr const char DefaultLocation[] = "autoware";

/// @brief Vehicle traffic rules for Autoware based on GermanVehicle, but allowing areas in normal
/// driving mode. GermanVehicle unconditionally disallows canPass(ConstArea), which prevents routing
/// through areas. This class restores the GenericTrafficRules behavior that checks participant
/// tags.
class AutowareVehicle : public traffic_rules::GermanVehicle
{
public:
  using GermanVehicle::GermanVehicle;

  // Allow passing through areas based on participant tags (GenericTrafficRules default behavior)
  bool canPass(const ConstArea & area) const override { return GenericTrafficRules::canPass(area); }
};

}  // namespace lanelet::autoware

#endif  // AUTOWARE_LANELET2_EXTENSION__TRAFFIC_RULES__AUTOWARE_TRAFFIC_RULES_HPP_
