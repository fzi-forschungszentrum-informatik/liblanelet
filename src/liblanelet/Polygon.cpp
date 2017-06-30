// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright (c) 2017, FZI Forschungszentrum Informatik
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Sebastian Klemm <klemm@fzi.de>
 *
 */
//----------------------------------------------------------------------

#include <boost/tuple/tuple.hpp>
#include "Polygon.hpp"

namespace LLet {

Polygon::Polygon()
{
}

Polygon::Polygon(const lanelet_ptr_t& lanelet)
{
  initialize(*lanelet);
}

Polygon::Polygon(const Lanelet& lanelet)
{
  initialize(lanelet);
}

Polygon::Polygon(const EventRegion& event_region)
{
  addVertices<std::vector<point_with_id_t>::const_iterator >(event_region.getVertices().begin(), event_region.getVertices().end());
}

void Polygon::initialize(const Lanelet& lanelet)
{
  const strip_ptr_t& left = boost::get<LLet::LEFT>(lanelet.bounds());
  const strip_ptr_t& right = boost::get<LLet::RIGHT>(lanelet.bounds());

  addVertices<std::vector<point_with_id_t>::const_iterator >(left->pts().begin(), left->pts().end());
  addVertices<std::vector<point_with_id_t>::const_reverse_iterator >(right->pts().rbegin(), right->pts().rend());
}

bool Polygon::pointInsidePolygon(const point_with_id_t& query) const
{
  if (m_vertices.size() < 3)
  {
    std::logic_error("Cannot work on polygon with too few points.");
  }

  const point_xy_t point = vec(m_reference_point, query);
  const std::size_t nr_vertices = m_vertices.size();

  std::size_t i, j;
  bool inside = false;
  for (i = 0, j = nr_vertices-1; i < nr_vertices; j = i++)
  {
    if (((boost::get<Y>(m_vertices[i]) > boost::get<Y>(point)) != (boost::get<Y>(m_vertices[j]) > boost::get<Y>(point))) &&
        (boost::get<X>(point) < (boost::get<X>(m_vertices[j]) - boost::get<X>(m_vertices[i]))
         * (boost::get<Y>(point) - boost::get<Y>(m_vertices[i]))
         / (boost::get<Y>(m_vertices[j])- boost::get<Y>(m_vertices[i])) + boost::get<X>(m_vertices[i])))
    {
      inside = !inside;
    }
  }
  return inside;
}

void Polygon::clear()
{
  m_vertices.clear();
}

double Polygon::distanceTo(const point_with_id_t &point) const
{
  point_xy_t metric_point = vec(m_reference_point, point);
  vertex_container_t vertices = m_vertices;
  assert(!vertices.empty());
  vertices.push_back(*vertices.begin());
  return dist(metric_point, projected(metric_point, vertices));
}

void Polygon::boundingBox(point_xy_t& lower_left, point_xy_t& upper_right) const
{
  assert(!m_vertices.empty());
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (vertex_container_t::const_iterator vertex = m_vertices.begin(); vertex < m_vertices.end(); ++vertex)
  {
    if (boost::get<X>(*vertex) < min_x)
    {
      min_x = boost::get<X>(*vertex);
    }

    if (boost::get<X>(*vertex) > max_x)
    {
      max_x = boost::get<X>(*vertex);
    }

    if (boost::get<Y>(*vertex) < min_y)
    {
      min_y = boost::get<Y>(*vertex);
    }

    if (boost::get<Y>(*vertex) > max_y)
    {
      max_y = boost::get<Y>(*vertex);
    }
  }

  boost::get<X>(lower_left) = min_x;
  boost::get<Y>(lower_left) = min_y;

  boost::get<X>(upper_right) = max_x;
  boost::get<Y>(upper_right) = max_y;
}

void Polygon::addVertex(const point_xy_t& point)
{
  m_vertices.push_back(point);
}

}
