^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_laser_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-06-17)
------------------

1.0.6 (2019-05-20)
------------------

1.0.5 (2019-05-06)
------------------

1.0.4 (2019-03-22)
------------------
* Merge branch 'tf2-frames' into 'erbium-devel'
  removed leading slash in sensors frames
  See merge request robots/pmb2_navigation!37
* removed leading slash in sensors frames
* Contributors: Procópio Stein

1.0.3 (2019-01-25)
------------------

1.0.2 (2019-01-17)
------------------

1.0.1 (2019-01-15)
------------------

1.0.0 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Specifics refactor
  See merge request robots/pmb2_navigation!30
* Expand all laser configurations
* rename laser launch files according to new standard
* Contributors: Victor Lopez

0.13.17 (2018-12-19)
--------------------
* reduced laser fov to avoid noise
* Contributors: Procópio Stein

0.13.16 (2018-11-21)
--------------------

0.13.15 (2018-10-20)
--------------------

0.13.14 (2018-10-03)
--------------------

0.13.13 (2018-09-28)
--------------------

0.13.12 (2018-09-26)
--------------------

0.13.11 (2018-09-26)
--------------------

0.13.10 (2018-09-17)
--------------------

0.13.9 (2018-06-22)
-------------------

0.13.8 (2018-05-17)
-------------------

0.13.7 (2018-05-15)
-------------------

0.13.6 (2018-04-24)
-------------------

0.13.5 (2018-04-17)
-------------------

0.13.4 (2018-04-12)
-------------------

0.13.3 (2018-04-06)
-------------------

0.13.2 (2018-03-08)
-------------------
* Merge branch 'restore-old-hokuyo-node' into 'dubnium-devel'
  Revert "replace hokuyo_node with urg_node"
  See merge request robots/pmb2_navigation!12
* Revert "replace hokuyo_node with urg_node"
  This reverts commit 97a9bbe24e1efbbca6cd59c54acd5b99bbc4ce7c.
* Contributors: Procópio Stein, Victor Lopez

0.13.1 (2018-02-15)
-------------------

0.13.0 (2018-02-01)
-------------------
* Merge branch 'urg-node-driver' into 'dubnium-devel'
  replace hokuyo_node with urg_node
  See merge request robots/pmb2_navigation!10
* replace hokuyo_node with urg_node
* Contributors: Procópio Stein

0.12.0 (2017-10-17)
-------------------

0.11.10 (2017-09-27)
--------------------
* added rgbd scan related files
* normalized package.xml for all packages
* Contributors: Procópio Stein

0.11.9 (2017-09-19)
-------------------

0.11.8 (2017-09-18)
-------------------

0.11.7 (2017-08-08)
-------------------
* updated launch params to match tiago's
* change default laser to sick_tim561
* cosmetic
* Contributors: Procópio Stein

0.11.6 (2017-07-03)
-------------------
* increased lasers fov
* Contributors: Procópio Stein

0.11.5 (2017-06-30)
-------------------

0.11.4 (2017-06-30)
-------------------

0.11.3 (2017-06-01)
-------------------

0.11.2 (2017-04-25)
-------------------

0.11.1 (2017-04-22)
-------------------
* moved filter launch to base launch
* added filter to hokuyo launch file
* Contributors: Procópio Stein

0.11.0 (2017-02-28)
-------------------
* 0.10.4
* changelogs
* Contributors: Procópio Stein

0.10.4 (2017-02-28)
-------------------

0.10.3 (2017-02-24)
-------------------

0.10.2 (2017-02-23)
-------------------
* added dependency to pal_filters
* Contributors: Procópio Stein

0.10.1 (2017-02-23)
-------------------
* removed rgbd related files
* replaced dependency of pal_laser_filters to laser_filters
* normalized and updated laser files
* fix sick laser launch files
* Contributors: Jeremie Deray, Procópio Stein

0.10.0 (2016-03-15)
-------------------
* load laser model on param srv
* Contributors: Jeremie Deray

0.9.15 (2016-03-10)
-------------------

0.9.14 (2016-03-02)
-------------------
* rm usuless deps rplidar
* Contributors: Jeremie Deray

0.9.13 (2016-02-10)
-------------------
* revert sick tim561 time offset
* Contributors: Jeremie Deray

0.9.12 (2016-02-10)
-------------------
* fixed time_offset for tim 561
* Contributors: Procopio Stein

0.9.11 (2016-02-09)
-------------------
* launch the laser based on argument "laser"
* added launch of tim571 and modified 551 for driver sick_tim
* Contributors: Sergio Ramos

0.9.10 (2016-02-09)
-------------------
* update pmb2 laser pkg.xml
* uses sick_tim pkg rather than old version
* added support for sick tim561
* Contributors: Jeremie Deray

0.9.9 (2015-10-26)
------------------

0.9.8 (2015-10-01)
------------------
* change hokuyo port
* laser.launch param to choose hokuyo or sick
* rm rebujito_laser
* Contributors: Jeremie Deray

0.9.7 (2015-02-02)
------------------
* Replace ant -> pmb2
* Rename files
* Contributors: Enrique Fernandez
