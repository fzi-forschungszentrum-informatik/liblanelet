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
# liblanelet
# -------------------------------------

ICMAKER_SET("liblanelet" IDE_FOLDER ${LIBLANELET_IDE_FOLDER})

FIND_PACKAGE(Pugixml)

ICMAKER_ADD_SOURCES(
  Attribute.cpp
  LaneletBase.cpp
  Lanelet.cpp
  LaneletMap.cpp
  LineStrip.cpp
  llet_xml.cpp
  LLTree.cpp
  Polygon.cpp
  RegulatoryElement.cpp
  Corridor.cpp
  Exceptions.cpp
  ParkingSpace.cpp
  EventRegion.cpp
  TrafficLight.cpp
  )

ICMAKER_ADD_HEADERS(
  Attribute.hpp
  BoundingBox.hpp
  convert_coordinates.hpp
  ImportExport.h
  LaneletBase.hpp
  LaneletFwd.hpp
  LaneletGraph.hpp
  Lanelet.hpp
  LaneletMap.hpp
  lanelet_point.hpp
  LineStrip.hpp
  llet_xml.hpp
  LLTree.hpp
  LocalGeographicCS.hpp
  normalize_angle.hpp
  EventRegion.hpp
  Polygon.hpp
  prettyprint.hpp
  RegulatoryElement.hpp
  RTree.h
  Corridor.hpp
  Exceptions.h
  ParkingSpace.hpp
  TrafficLight.hpp
  )

ICMAKER_LOCAL_CPPDEFINES(-DLIBLANELET_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_LIBLANELET_)
ICMAKER_INCLUDE_DIRECTORIES(${LIBLANELET_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  Boost
  tinyxml
  )

ICMAKER_DEPENDENCIES(
  Pugixml
  )

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(liblanelet)




# -------------------------------------
# lanelet_demo
# -------------------------------------

ICMAKER_SET("lanelet_demo")

ICMAKER_ADD_SOURCES(
  demo.cpp
  )

ICMAKER_DEPENDENCIES(
  liblanelet
)

ICMAKER_BUILD_PROGRAM()
