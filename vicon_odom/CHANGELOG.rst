^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vicon_odom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2017-12-08)
------------------
* Find Eigen3 using native CMake module to avoid warnings.
  CMake version required is now 3.0.0 which means Ubuntu 14.04 will not
  work out of the box. Trusty ships with CMake 2.8.
* Publish PoseStamped in addition to PoseWithCovarianceStamped
  NOTE: Launch file remaps will have to be changed: pose is now pose_cov
* Contributors: Alex Spitzer

0.0.2 (2017-08-04)
------------------

0.0.1 (2017-03-04)
------------------
* Warning in vicon odom when vicon data period 50% higher than expected
* Add hack because mavros expects PoseStamped (without covariance)
* Add ability to inject noise into pose position and report covariance
* added missing lines to vicon CMakeLists.txt, removed unnecessary add_dependencies from vicon_odom CMakeLists.txt
* add a geometry_msgs/PoseStamped publisher, fix bug in CMakelists.txt
* publish an invalid all-zero quaternion if less than min_visible_markers markers are visible, add descriptions to static variables
* modified Kalman filter to not publish odometry whenever the number of visible markers falls below a user defined threshold, modified launch files for RASL settings
* Adding mandatory frame_id params
* Fixes construction of tf broadcaster before init. Copies identity yaml. Copies debug lib for OSX
* Adding tf2 broadcaster of vicon pose
* Ensure vicon msgs are generated prior to building vicon_odom
* Restructure, hydro support, move to cmake build
* Contributors: Alex Spitzer, John Yao, Nathan Michael
