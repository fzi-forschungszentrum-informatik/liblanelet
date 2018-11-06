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
 * \author  Philip Schoerner <schoerner@fzi.de>
 * \author  Jens Doll <doll@fzi.de>
 * \date    2018-04-03
 *
 */
//----------------------------------------------------------------------

#include "TrafficLight.hpp"
#include "Polygon.hpp"

namespace LLet
{

TrafficLight::TrafficLight(const std::vector<point_with_id_t>& vertices)
{
  _vertices = vertices;
  assert(isValid());
  _polygon.reset(new Polygon(vertices));
}

TrafficLight::~TrafficLight ()
{
}

bool TrafficLight::isValid()
{
  return (_vertices.size() >= 3) && (_vertices.front().get<2>() == _vertices.back().get<2>());
}

const int64_t TrafficLight::getIntersectionId() const
{
  return _intersection_id;
}

const int64_t TrafficLight::getSignalGroupId() const
{
  return _signalgroup_id;
}

const int64_t TrafficLight::getId() const
{
  return _id;
}

void TrafficLight::setId(const int64_t id)
{
  _id = id;
}

void TrafficLight::setIds(const int64_t intersection_id, const int64_t signalgroup_id)
{
  _intersection_id = intersection_id;
  _signalgroup_id = signalgroup_id;
}

const TrafficLight::Type TrafficLight::getType() const
{
  return _event_type;
}

void TrafficLight::setType(const Type value)
{
  _event_type = value;
}

const std::vector< point_with_id_t >& TrafficLight::getVertices() const
{
  return _vertices;
}

const std::vector< regulatory_element_ptr_t >& TrafficLight::getRegulatoryElements() const
{
  return _regulatory_elements;
}

void TrafficLight::addRegulatoryElement(regulatory_element_ptr_t element)
{
  _regulatory_elements.push_back(element);
}


} // namespace
