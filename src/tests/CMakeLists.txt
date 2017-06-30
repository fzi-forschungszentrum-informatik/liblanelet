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

