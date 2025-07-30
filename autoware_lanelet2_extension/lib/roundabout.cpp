// Copyright 2023 TIER IV, Inc.
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

#include "autoware_lanelet2_extension/regulatory_elements/roundabout.hpp"

#include <boost/variant.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace lanelet::autoware
{
namespace
{
template <typename T>
bool findAndErase(const T & primitive, RuleParameters * member)
{
  if (member == nullptr) {
    std::cerr << __FUNCTION__ << ": member is null pointer";
    return false;
  }
  auto it = std::find(member->begin(), member->end(), RuleParameter(primitive));
  if (it == member->end()) {
    return false;
  }
  member->erase(it);
  return true;
}

template <typename T>
Optional<T> tryGetFront(const std::vector<T> & vec)
{
  if (vec.empty()) {
    return {};
  }
  return vec.front();
}

template <typename T>
RuleParameters toRuleParameters(const std::vector<T> & primitives)
{
  auto cast_func = [](const auto & elem) { return static_cast<RuleParameter>(elem); };
  return utils::transform(primitives, cast_func);
}

RegulatoryElementDataPtr constructRoundabout(
  Id id, const AttributeMap & attributes, const lanelet::Lanelets & roundabout_entry_lanelets,
  const lanelet::Lanelets & roundabout_exit_lanelets,
  const lanelet::Lanelets & roundabout_internal_lanelets)
{
  RuleParameterMap rpm;

  for (const auto & entry : roundabout_entry_lanelets) {
    RuleParameters rule_parameters = {RuleParameter(entry)};
    rpm.insert(std::make_pair(Roundabout::AutowareRoleNameString::Entry, rule_parameters));
  }

  for (const auto & exit : roundabout_exit_lanelets) {
    RuleParameters rule_parameters = {RuleParameter(exit)};
    rpm.insert(std::make_pair(Roundabout::AutowareRoleNameString::Exit, rule_parameters));
  }
  for (const auto & internal : roundabout_internal_lanelets) {
    RuleParameters rule_parameters = {RuleParameter(internal)};
    rpm.insert(std::make_pair(Roundabout::AutowareRoleNameString::Internal, rule_parameters));
  }
  auto data = std::make_shared<RegulatoryElementData>(id, std::move(rpm), attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = "roundabout";
  return data;
}
}  // namespace

Roundabout::Roundabout(const RegulatoryElementDataPtr & data) : RegulatoryElement(data)
{
}

Roundabout::Roundabout(
  Id id, const AttributeMap & attributes, const lanelet::Lanelets & roundabout_entry_lanelets,
  const lanelet::Lanelets & roundabout_exit_lanelets,
  const lanelet::Lanelets & roundabout_internal_lanelets)
: Roundabout(constructRoundabout(
    id, attributes, roundabout_entry_lanelets, roundabout_exit_lanelets,
    roundabout_internal_lanelets))
{
}

lanelet::ConstLanelets Roundabout::roundaboutLanelets() const
{
  lanelet::ConstLanelets lanelets;
  auto entry_lanelets = getParameters<lanelet::ConstLanelet>(AutowareRoleNameString::Entry);
  auto exit_lanelets = getParameters<lanelet::ConstLanelet>(AutowareRoleNameString::Exit);
  auto internal_lanelets = getParameters<lanelet::ConstLanelet>(AutowareRoleNameString::Internal);

  lanelets.insert(lanelets.end(), entry_lanelets.begin(), entry_lanelets.end());
  lanelets.insert(lanelets.end(), exit_lanelets.begin(), exit_lanelets.end());
  lanelets.insert(lanelets.end(), internal_lanelets.begin(), internal_lanelets.end());

  return lanelets;
}

lanelet::ConstLanelets Roundabout::roundaboutEntryLanelets() const
{
  return getParameters<lanelet::ConstLanelet>(AutowareRoleNameString::Entry);
}

lanelet::ConstLanelets Roundabout::roundaboutExitLanelets() const
{
  return getParameters<lanelet::ConstLanelet>(AutowareRoleNameString::Exit);
}
lanelet::ConstLanelets Roundabout::roundaboutInternalLanelets() const
{
  return getParameters<lanelet::ConstLanelet>(AutowareRoleNameString::Internal);
}

bool Roundabout::isEntryLanelet(const lanelet::ConstLanelet & lanelet) const
{
  const auto entries = roundaboutEntryLanelets();
  return std::any_of(
    entries.begin(), entries.end(), [&](const auto & l) { return l.id() == lanelet.id(); });
}

bool Roundabout::isExitLanelet(const lanelet::ConstLanelet & lanelet) const
{
  const auto exits = roundaboutExitLanelets();
  return std::any_of(
    exits.begin(), exits.end(), [&](const auto & l) { return l.id() == lanelet.id(); });
}

RegisterRegulatoryElement<Roundabout> regRoundabout;

}  // namespace lanelet::autoware

// NOLINTEND(readability-identifier-naming)
