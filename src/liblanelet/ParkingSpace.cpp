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
 * \author  Tim Pollert <pollert@fzi.de>
 *
 */
//----------------------------------------------------------------------

#include "ParkingSpace.hpp"

namespace LLet {

ParkingSpace::ParkingSpace(const std::vector<point_with_id_t>& _vertices,
                           const ParkingDirection _parking_direction) :
  vertices(_vertices),
  parking_direction(_parking_direction)
{
}

ParkingSpace::~ParkingSpace()
{
}

bool ParkingSpace::isValid() const
{
  return vertices.size() == numberOfExpectedPoints()
      && (vertices.front().get<2>() == vertices.back().get<2>());
}

const int32_t ParkingSpace::getId() const
{
  return id;
}

void ParkingSpace::setId(const int32_t value)
{
  id = value;
}

const std::vector< point_with_id_t >& ParkingSpace::getVertices() const
{
  return vertices;
}

const point_with_id_t ParkingSpace::getCenterPoint() const
{
  point_with_id_t center_point; //initialized with 0,0,-1
  for (unsigned int i = 0; i < vertices.size()-1; ++i) //last point redundant
  {
    boost::get<LLet::LAT>(center_point) += boost::get<LLet::LAT>(vertices[i]);
    boost::get<LLet::LON>(center_point) += boost::get<LLet::LON>(vertices[i]);
  }

  const float num_vertices = float(vertices.size()-1);
  boost::get<LLet::LAT>(center_point) /= num_vertices;
  boost::get<LLet::LON>(center_point) /= num_vertices;
  return center_point;
}

const ParkingSpace::ParkingDirection ParkingSpace::getParkingDirection() const
{
  return parking_direction;
}

const std::vector< int32_t >& ParkingSpace::getRelatedLanelets() const
{
  return related_lanelets;
}

void ParkingSpace::addRelatedLanelet(const int32_t _related_lanelet)
{
  related_lanelets.push_back(_related_lanelet);
}


} // END NAMESPACE
