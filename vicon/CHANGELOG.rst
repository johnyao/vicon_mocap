^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vicon
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2017-12-08)
------------------
* Remove Boost from vicon CMakeLists catkin_package depends
  Boost is not needed in the headers. This also has the benefit of
  removing the cmake Boost warning.
* Find Eigen3 using native CMake module to avoid warnings.
  CMake version required is now 3.0.0 which means Ubuntu 14.04 will not
  work out of the box. Trusty ships with CMake 2.8.
* Contributors: Alex Spitzer

0.0.2 (2017-08-04)
------------------
* Fix metapackage directory structure to work with newer catkin tools
  Packages should not be nested.
  Make libvicon_driver (now vicon_driver) its own package so that vicon
  package can now depend on it. There is a silly hack needed to find the
  boost libraries needed by the ViconSDK.
  This should now be properly built using all versions of catkin tools.
* Contributors: Alex Spitzer

0.0.1 (2017-03-04)
------------------
* added missing lines to vicon CMakeLists.txt, removed unnecessary add_dependencies from vicon_odom CMakeLists.txt
* added calibration stand files
* modified Kalman filter to not publish odometry whenever the number of visible markers falls below a user defined threshold, modified launch files for RASL settings
* Fixes construction of tf broadcaster before init. Copies identity yaml. Copies debug lib for OSX
* Restructure, hydro support, move to cmake build
* Contributors: John Yao, Nathan Michael
