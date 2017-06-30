# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# -- END LICENSE BLOCK ------------------------------------------------

#----------------------------------------------------------------------
# \file
#
# \author  Florian Kuhnt <kuhnt@fzi.de>
# \date    2017-01-01
#
#----------------------------------------------------------------------


CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# Include icmaker if not already available.
#
# When building in an ic_workspace based on icmaker, icmaker is usually globally
# available.
#
# When using catkin (for example in ROS workspaces), icmaker is not globally
# available and thus has to be found and included in every package. Also catkin
# packages usually define a project per package.
#
#
IF(NOT DEFINED ICMAKER_DIRECTORY)
  find_package(icmaker REQUIRED)
  INCLUDE(UseFullIcMaker REQUIRED)
  project(liblanelet)
ENDIF()


# To be used by other modules
SET(LIBLANELET_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}/src" CACHE INTERNAL "")
SET(LIBLANELET_IDL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(LIBLANELET_IDE_FOLDER "liblanelet")

ICMAKER_REGISTER_PACKAGE(liblanelet)

ADD_SUBDIRECTORY(src/liblanelet)
ADD_SUBDIRECTORY(src/tools)


ICMAKER_CONFIGURE_PACKAGE()

IF(BUILD_TESTS AND NOT WIN32)
  ADD_SUBDIRECTORY(src/tests/)
ENDIF()
