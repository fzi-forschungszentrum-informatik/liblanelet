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
# lanelet_tools
# -------------------------------------

ICMAKER_SET("lanelet_to_CarMaker")

ICMAKER_ADD_SOURCES(
   Lanelet2CarMaker.cpp
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


#################################################

ICMAKER_SET("lanelet_graph_to_graphviz")

ICMAKER_ADD_SOURCES(
  lanelet_graph_to_graphviz.cpp
)

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(EXPORT
  Boost
  liblanelet
  icl_core_config
  icl_core_logging
)

ICMAKER_BUILD_PROGRAM()


#################################################

ICMAKER_SET("lanelet_graph_to_graphml")

ICMAKER_ADD_SOURCES(
  lanelet_graph_to_graphml.cpp
)

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(EXPORT
  Boost
  liblanelet
  icl_core_config
  icl_core_logging
)

ICMAKER_BUILD_PROGRAM()


#################################################


ICMAKER_SET("lanelet_graph_to_osm")

ICMAKER_ADD_SOURCES(
   lanelet_graph_to_osm.cpp
)

IF(CMAKE_COMPILER_IS_GNUCXX)
  ICMAKER_GLOBAL_CPPDEFINES(-DCMAKE_CXX_FLAGS "-std=c++0x")
ENDIF()

ICMAKER_DEPENDENCIES(EXPORT
  Boost
)

ICMAKER_DEPENDENCIES(
  liblanelet
  icl_core_config
  icl_core_logging
)

ICMAKER_BUILD_PROGRAM()
