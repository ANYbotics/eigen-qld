# eigen-qld

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/eigen-qld/doxygen/HEAD/index.html)

eigen-qld provides an interface to use the [QLD QP solver](https://help.scilab.org/doc/5.5.2/en_US/qld.html) with the [Eigen3](https://eigen.tuxfamily.org) library.

This package has been forked from [jrl-umi3218](https://github.com/jrl-umi3218/eigen-qld). The following changes were made:

- Catkin packaging for ROS
- Removed F2C option (slower) and Python bindings
- Unit tests converted from Boost to Google Test

## Installation

### Install Debian Packages

To install all packages from the this repository as Debian packages use

    sudo apt install ros-${ROS_DISTRO}-eigen-qld

### Building from Source

#### Dependencies

- [CMake](cmake.org) >= 3.5
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2
- [Robot Operating System (ROS)](http://wiki.ros.org)
- [g++](https://gcc.gnu.org/)
- [gfortran](https://gcc.gnu.org/fortran/)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ~/catkin_ws/src
    git clone git@github.com:ANYbotics/eigen-qld.git
    catkin build eigen-qld

### Unit Tests

Run the unit tests with

    catkin run_tests eigen-qld
