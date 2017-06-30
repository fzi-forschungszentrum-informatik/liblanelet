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

#ifndef PARKINGSPACE_HPP
#define PARKINGSPACE_HPP

// Boost includes
#include <boost/shared_ptr.hpp>

// Lanelet includes
#include "Polygon.hpp"
#include "Attribute.hpp"


namespace LLet
{

class ParkingSpace;

typedef boost::shared_ptr< ParkingSpace >  parking_space_ptr_t;



/*! A parking space labels an area where parking is allowed.
 *  The parking space data structure requires 5 vertices.
 *  The first and last vertex must be the same.
 */
class ParkingSpace
{
public:

  enum ParkingDirection {
    FORWARD,
    BACKWARD,
    BOTH
  };

  //! Number of points needed to create a parking space.
  static std::size_t numberOfExpectedPoints()
  {
    return 5;
  }

  //! Constructor
  ParkingSpace(const std::vector<point_with_id_t>& _vertices, const ParkingDirection _parking_direction);

  //! Destructor
  ~ParkingSpace();

  /*! \return true if there are \see number_of_expected_points
   *  points and the first and last point are the same.
   */
  bool isValid() const;

  //! Getter id
  const int32_t getId() const;

  //! Setter id
  void setId(const int32_t value);

  //! Getter vertices
  const std::vector< point_with_id_t >& getVertices() const;

  //! Get the center point of parking space
  const point_with_id_t getCenterPoint() const;

  //! Getter Parking Direction
  const ParkingDirection getParkingDirection() const;

  //! Getter Related Lanelets. A parking space is related to a lanelet
  //! if parking space can be accessed via a lanelet.
  const std::vector< int32_t >& getRelatedLanelets() const;

  //! Add a related lanelet
  void addRelatedLanelet(const int32_t _related_lanelet);

private:
  //! Identification number of the parking space
  int32_t id;

  //! The defining points of the parking space area.
  std::vector< point_with_id_t > vertices;

  //! Allowed parking direction (FORWARD, BACKWARD or BOTH).
  ParkingDirection parking_direction;

  //! The parking space is accessible from this lanelets.
  std::vector< int32_t > related_lanelets;
};

}
#endif // PARKINGSPACE_HPP
