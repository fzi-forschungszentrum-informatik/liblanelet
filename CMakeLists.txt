# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright (c) 2018, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
