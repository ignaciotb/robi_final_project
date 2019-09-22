^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_detection_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.12.13 (2019-03-25)
--------------------

0.12.12 (2019-02-05)
--------------------

0.12.11 (2019-01-22)
--------------------
* Merge branch 'object_recognition_msgs' into 'dubnium-devel'
  added object_recognition msgs for inference_server
  See merge request common/pal_msgs!22
* added object_recognition msgs for inference_server
* Contributors: Sai Kishor Kothakota, Victor Lopez

0.12.10 (2018-12-18)
--------------------

0.12.9 (2018-11-16)
-------------------

0.12.8 (2018-10-31)
-------------------

0.12.7 (2018-06-01)
-------------------

0.12.6 (2018-01-17)
-------------------

0.12.5 (2018-01-16)
-------------------

0.12.4 (2018-01-12)
-------------------

0.12.3 (2017-10-06)
-------------------
* copy message from pal_vision_msgs to pal_detection
* Contributors: Jordi Pages

0.12.2 (2017-09-29)
-------------------

0.12.1 (2017-05-26)
-------------------

0.12.0 (2017-01-13)
-------------------
* add gender fields. Fixes #15063.
* Contributors: Jordi Pages

0.11.6 (2016-12-14)
-------------------

0.11.5 (2016-12-02)
-------------------

0.11.4 (2016-10-10)
-------------------

0.11.3 (2016-10-07)
-------------------

0.11.2 (2016-09-19)
-------------------

0.11.1 (2016-07-11)
-------------------

0.11.0 (2016-07-11)
-------------------
* add emotions fields
* Contributors: Jordi Pages

0.10.4 (2015-09-04)
-------------------
* add facial expressions and confidence information
* Contributors: Jordi Pages

0.10.3 (2015-03-09)
-------------------

0.10.2 (2015-02-06)
-------------------

0.10.1 (2014-11-17)
-------------------

0.9.1 (2014-05-27)
------------------
* Modifications for the pal_person_detection_fuser
* add size of object in SelectTexturedObject
  and add comments on both messages
* add services for textured object detection
* image included in the message is now compressed
* pal_detection_msgs: fix message generation
* Added other packages needed by people that want to use our robot, face
  detection in pal_detection_msgs, and text to speech in text_to_speech. Also
  removed from pal_interaction_msgs the references to the speech part that was
  included there and made incompatible the use of axclient without having the
  same package name than the one inside of the real robot
* Contributors: Jordi Pages, Paul Mathieu, Sammy Pfeiffer
