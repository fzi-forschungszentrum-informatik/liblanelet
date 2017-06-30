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
