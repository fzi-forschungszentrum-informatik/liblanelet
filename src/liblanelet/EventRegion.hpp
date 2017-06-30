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
* \date    2016-05-23
*
*/
//----------------------------------------------------------------------

#pragma once

// Boost includes
#include <vector>
#include <boost/shared_ptr.hpp>

// Lanelet includes
#include "lanelet_point.hpp"

namespace LLet
{

class Polygon;

class LANELET_IMPORT_EXPORT EventRegion
{
public:

  enum Type
  {
    INTERSECTION,
    PARKING,
    UNKNOWN
  };

  /*! Constructor for an event region.
    * \param vertices must be 5 points and the first and last
    * point has to be the same.
    */
  EventRegion(const std::vector<point_with_id_t>& vertices);

  //! Destructor
  ~EventRegion();

  //! Check if the region is valid.
  bool isValid();

  //! Getter id
  const int32_t getId() const;

  //! Setter id
  void setId(const int32_t value);

  //! Getter event type
  const Type getType() const;

  //! Setter event type
  void setType(const Type value);

  //! Getter vertices
  const std::vector< point_with_id_t >& getVertices() const;

  /*! Determine if the event region, when interpreted as a polygon's area,
     *  is covering the given point's coordinates.
     * \param query The point to check
     * \return \c true, if so, \c else otherwise
     */
  bool covers_point(const point_with_id_t& query) const;


private:
  //! Identification number of the event region
  int32_t _id;

  //! Type of the event region e.g. intersection etc.
  Type  _event_type;

  //! The defining points of the event region.
  std::vector< point_with_id_t > _vertices;

  //! The event region converted into a polygon.
  boost::shared_ptr<Polygon> _polygon;
};

typedef boost::shared_ptr< EventRegion >  event_region_ptr_t;

} // namespace


