^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2018-10-24)
------------------
* Merge branch 'improve-constness' into 'erbium-devel'
  pal_statistics changed API
  See merge request navigation/robot_pose!7
* pal_statistics changed API
* Contributors: Victor Lopez

1.0.1 (2018-10-23)
------------------
* Merge branch 'tf2-migration-erbium' into 'erbium-devel'
  Tf2 migration erbium
  See merge request navigation/robot_pose!5
* catch as &
* added license
* added robot_pose test
  Conflicts:
  CMakeLists.txt
* do not publish if there is a tf exception
* added missing dependencies
* reduced freq to 50
* split tf lookup into two, to avoid jumps due to amcl initialpose
  this happened when slippaged detector published several initialpose
  messages. we found out that splitting the lookup from
  map -> base_footprint to map -> odom -> base_footprint
  solved the issue
* migration from tf to tf2
* Contributors: Procópio Stein, Victor Lopez

1.0.0 (2018-07-18)
------------------
* Merge branch 'add-distance-travelled' into 'erbium-devel'
  Add distance travelled publisher
  See merge request navigation/robot_pose!4
* Replace amcl_pose with robot_pose
* Add distance travelled publisher
* cosmetic, matching pal code style
* upgrade package to format 2, cleaned cmakelists
* Contributors: Procópio Stein, Victor Lopez

0.0.2 (2016-03-04)
------------------
* Merge branch 'creation' into 'master'
  changed publisher queue size and fixed authors name
  fixed queue size and authors name
  See merge request !1
* changed publisher queue size and fixed authors name
* fixed tranform direction
* robot pose node implemented
* added readme
* Contributors: Procopio Stein, Victor Lopez
