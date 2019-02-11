^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vicon_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2019-02-11)
------------------

0.0.3 (2017-12-08)
------------------
* Find Eigen3 using native CMake module to avoid warnings.
  CMake version required is now 3.0.0 which means Ubuntu 14.04 will not
  work out of the box. Trusty ships with CMake 2.8.
* vicon_calib.cpp: Fix warning from -Wall
* Contributors: Alex Spitzer

0.0.2 (2017-08-04)
------------------
* Install vicon_driver headers as well as required boost libs
  This was causing compilation failures when installing to install.
* Fix metapackage directory structure to work with newer catkin tools
  Packages should not be nested.
  Make libvicon_driver (now vicon_driver) its own package so that vicon
  package can now depend on it. There is a silly hack needed to find the
  boost libraries needed by the ViconSDK.
  This should now be properly built using all versions of catkin tools.
* Contributors: Alex Spitzer

0.0.1 (2017-03-04)
------------------
