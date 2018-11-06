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


#################################################


ICMAKER_SET("lanelet_checker")

ICMAKER_ADD_SOURCES(
  lanelet_checker.cpp
  )

ICMAKER_DEPENDENCIES(
  liblanelet
  icl_core_config
  icl_core_logging
)

ICMAKER_BUILD_PROGRAM()
