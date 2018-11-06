// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright (c) 2018, FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of
//    conditions and the following disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific prior written
//    permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Sebastian Klemm <klemm@fzi.de>
 *
 */
//----------------------------------------------------------------------

#pragma once

#include <vector>
#include <iterator>
#include "ImportExport.h"
#include "lanelet_point.hpp"
#include "LineStrip.hpp"
#include "Lanelet.hpp"
#include "EventRegion.hpp"

namespace LLet
{

class LANELET_IMPORT_EXPORT Polygon
{
public:

  //! Constructor
  Polygon();

  //! Constructor using line strip information from the given \a lanelet
  Polygon(const lanelet_ptr_t& lanelet);

  //! Constructor using line strip information from the given \a lanelet
  Polygon(const Lanelet& lanelet);

  //! Constructor using event region infotmation form the given \a eventRegion
  Polygon(const EventRegion& event_region);

  //! Constructor using a vector of vertices
  Polygon(const std::vector<point_with_id_t>& vertices);

  /*! Check if \a query lies within the polygon's area.
   *  \throws std::logic_error if called on polygon with too few points
   */
  bool pointInsidePolygon(const point_with_id_t& query) const;

  double distanceTo(const point_with_id_t& point) const;

  const vertex_container_t& vertices() const
  {
    return m_vertices;
  }

  //! Calculate the bounding box surrounding the polygon
  void boundingBox(point_xy_t& lower_left, point_xy_t& upper_right) const;

  const point_with_id_t gnssReferencePoint() const
  {
    return m_reference_point;
  }

private:
  void initialize(const Lanelet& lanelet);

  //! Add a \a point in local metric cs to the polygon
  void addVertex(const point_xy_t& point);

  //! Clear all stored data
  void clear();

  //! Add vertices by iterating from \a begin_it to \a end_it
  template <typename input_iterator_t>
  void addVertices(const input_iterator_t& begin_it,
                   const input_iterator_t& end_it);

  /*! Reference to convert between gnss <-> local cs.
   *  To create a metric polygon the first available
   *  point_with_id_t will be used.
   */
  point_with_id_t m_reference_point;

  /*! Vertex storage.
   *  All contained vertices are considered connected in
   *  the existing order
   */
  vertex_container_t m_vertices;

};


template <typename input_iterator_t>
void Polygon::addVertices(const input_iterator_t& begin_it,
                          const input_iterator_t& end_it)
{
  if (m_vertices.empty())
  {
    m_reference_point = (*begin_it);
  }
  for (input_iterator_t it = begin_it; it != end_it; ++it)
  {
    addVertex(vec(m_reference_point, *it));
  }
}

}
