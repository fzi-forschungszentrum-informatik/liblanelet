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


# -------------------------------------
# lanelet_test
# -------------------------------------

ICMAKER_SET("lanelet_test")

# todo: rather use FIND_PACKAGE() instead..
FIND_PATH(GTest_INCLUDE_DIRS NAMES gtest/gtest.h PATHS /usr/include)
FIND_PATH(GTest_SOURCE_DIR NAMES src/gtest.cc PATHS /usr/src/gtest)

INCLUDE_DIRECTORIES(${GTest_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  ${GTest_SOURCE_DIR}/src/gtest.cc
  ${GTest_SOURCE_DIR}/src/gtest-death-test.cc
  ${GTest_SOURCE_DIR}/src/gtest-filepath.cc
  ${GTest_SOURCE_DIR}/src/gtest-port.cc
  ${GTest_SOURCE_DIR}/src/gtest-printers.cc
  ${GTest_SOURCE_DIR}/src/gtest-test-part.cc
  ${GTest_SOURCE_DIR}/src/gtest-typed-test.cc
  lanelet_tests.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(EXPORT
  Boost
  Pthread
  )

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()



# -------------------------------------
# linestrip_interpol_test
# -------------------------------------

ICMAKER_SET("linestrip_interpol_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  linestrip_interpol_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()


# -------------------------------------
# lanelet_polygon_test
# -------------------------------------

ICMAKER_SET("lanelet_polygon_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  lanelet_polygon_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()


ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()


# -----------------------------------------
# lanelet_distance_test (signed distances)
# -----------------------------------------

ICMAKER_SET("lanelet_distance_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  lanelet_distance_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()


# -----------------------------------------
# lanelet_distance_test (absolute distances)
# -----------------------------------------

ICMAKER_SET("lanelet_absolute_distance_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  lanelet_absolute_distance_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()

# -----------------------------------------
# lanelet_read_and_write_test
# -----------------------------------------

ICMAKER_SET("lanelet_read_and_write_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  lanelet_read_and_write_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()



# -----------------------------------------
# corridor_test (absolute distances)
# -----------------------------------------

ICMAKER_SET("lanelet_corridor_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  corridor_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()


# -----------------------------------------
# parking_space_test
# -----------------------------------------

ICMAKER_SET("parking_space_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  parking_space_test.cpp
  )

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
  )

ICMAKER_BUILD_PROGRAM()

# -----------------------------------------
# event_region_test
# -----------------------------------------

ICMAKER_SET("event_region_test")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../..)

ICMAKER_ADD_SOURCES(
  event_region_test.cpp
)

ICMAKER_ADD_HEADERS(
  )

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(
  liblanelet
)

ICMAKER_BUILD_PROGRAM()

