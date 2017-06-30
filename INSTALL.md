# Installation

There are three different installation approaches:
* [Linux system, e.g. Ubuntu 16.04](#install-on-linux-system-eg-ubuntu-1604)
* [Linux system with ROS, e.g. Ubuntu 16.04 with ROS kinetic](#install-on-linux-system-with-ros-eg-ubuntu-1604-with-ros-kinetic)
* [Linux system using icmaker standalone](#install-on-linux-system-using-icmaker-standalone)


## Install on Linux system, e.g. Ubuntu 16.04

On generic linux systems you have to build the core packages by hand:

### Install external dependencies

```bash
 sudo apt-get install libpugixml-dev libtinyxml-dev
```

### Build icmaker

```bash
git clone https://github.com/fzi-forschungszentrum-informatik/icmaker.git
mkdir icmaker/build
cd icmaker/build
cmake .. -DCMAKE_INSTALL_PREFIX=../export
make install
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$PWD/../export
```

### Build liblanelet

```bash
git clone https://github.com/fzi-forschungszentrum-informatik/liblanelet.git
mkdir liblanelet/build
cd liblanelet/build
cmake .. -DCMAKE_INSTALL_PREFIX=../export
make install
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$PWD/../export
```

Some of the tools are not built (because of missing icl_core dependencies). These are only available using the [icmaker standalone building approach](#linux-system-using-icmaker-standalone).

### Run examples

```bash
cd liblanelet
build/bin/lanelet_test
build/bin/lanelet_demo share/osm/sample.osm 
```

## Install on Linux system with ROS, e.g. Ubuntu 16.04 with ROS kinetic

Thanks to the build system *catkin* you can skip lots of the manual steps:

Install external dependencies: `sudo apt-get install libpugixml-dev libtinyxml-dev`

Create a catkin_ws

Clone icmaker and liblanelet into `catkin_ws/src` dir.

Build it with `catkin_make`

Some of the tools are not built (because of missing icl_core dependencies). These are only available using the [icmaker standalone building approach](#linux-system-using-icmaker-standalone).


## Install on Linux system using icmaker standalone

Using the icmaker standalone approach you can also build the icl_core-dependent tools. See https://github.com/fzi-forschungszentrum-informatik/icmaker#building-standalone for more information.
