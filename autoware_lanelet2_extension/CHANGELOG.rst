^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lanelet2_extension
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#11 <https://github.com/youtalk/autoware_lanelet2_extension/issues/11>`_ from youtalk/import-update
  feat: import updates from `autoware_common`
* fix link
* feat(lanelet2_extension): overwriteLaneletsCenterline supports "waypoints" (`#252 <https://github.com/youtalk/autoware_lanelet2_extension/issues/252>`_)
  * feat(lanelet2_extension): centerline is converted to waypoints
  * fix lanelet2_extension_python
  * update README
  * fix
  * fix
  * early return
  * fix clang-tidy
  * Update tmp/lanelet2_extension/lib/utilities.cpp
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  * style(pre-commit): autofix
  * fix
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: boost optional build error on rolling environment (`#241 <https://github.com/youtalk/autoware_lanelet2_extension/issues/241>`_)
* perf(lanelet2_extension): use std::unordered_set<>::find instead of std::find (`#244 <https://github.com/youtalk/autoware_lanelet2_extension/issues/244>`_)
  perf(exists): use std::unordered_set<>::find instead of std::find
* Contributors: Maxime CLEMENT, Takayuki Murooka, Yutaka Kondo, ぐるぐる

0.3.0 (2024-05-31)
------------------
* Merge remote-tracking branch 'upstream/main' into rolling
* Merge pull request `#6 <https://github.com/youtalk/autoware_lanelet2_extension/issues/6>`_ from youtalk/autoware-msgs-migration
  feat: `autoware_msgs` migration
* feat(autoware_common): update and replace autoware_auto_msg
* feat(lanelet2_extension): replace autoware_auto_mapping_msg with autoware_map_msg (`#216 <https://github.com/youtalk/autoware_lanelet2_extension/issues/216>`_)
  * feat(lanelet2_extension): replace autoware-auto-mapping-msg to autoware-map-msg
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yutaka Kondo, cyn-liu, liu cui

0.2.0 (2024-05-07)
------------------
* refactor: add `autoware\_` prefix to package names and namespaces (`#3 <https://github.com/youtalk/autoware_lanelet2_extension/issues/3>`_)
  * fix readme
  * fix link
  * fix link
  * revert readme
  * wip
  * update package names
  * update package names
  * fix include
  * rename module
  * rename to autoware\_
  * fix depend
  * rename to autoware\_
  * Revert "wip"
  This reverts commit 8079660c318feace9d841aacd39a8945aa144cf7.
  ---------
* Contributors: Yutaka Kondo
