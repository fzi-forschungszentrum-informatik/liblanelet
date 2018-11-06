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
#ifndef LIBLANELET_TRAFFICLIGHT_H_INCLUDED
#define LIBLANELET_TRAFFICLIGHT_H_INCLUDED

#include "RegulatoryElement.hpp"

namespace LLet
{

class Polygon;

class LANELET_IMPORT_EXPORT TrafficLight
{
public:

  enum Type
  {
    ALL,
    LEFT_ONLY,
    RIGHT_ONLY,
    STRAIGHT_ONLY,
    STRAIGHT_LEFT_ONLY,
    STRAIGHT_RIGHT_ONLY,
    UNKNOWN
  };

  /*! Constructor for an event region.
    * \param vertices must be 5 points and the first and last
    * point has to be the same.
    */
  TrafficLight(const std::vector<point_with_id_t>& vertices);

  //! Destructor
  ~TrafficLight();

  //! Check if the region is valid.
  bool isValid();

  //! Getter id
  const int64_t getId() const;
  const int64_t getIntersectionId() const;
  const int64_t getSignalGroupId() const;

  //! Setter id
  void setId(const int64_t id);
  void setIds(const int64_t intersection_id, const int64_t signalgroup_id);

  //! Getter event type
  const Type getType() const;

  //! Setter event type
  void setType(const Type value);

  //! Getter vertices
  const std::vector< point_with_id_t >& getVertices() const;

  //! Get all regulatory elements
  const std::vector< regulatory_element_ptr_t >& getRegulatoryElements() const;

  //! add regulatory element
  void addRegulatoryElement(regulatory_element_ptr_t element);


private:
  //! Identification number of the event region
  int64_t _id;
  int64_t _intersection_id;
  int64_t _signalgroup_id;

  //! Type of the event region e.g. intersection etc.
  Type  _event_type;

  //! The defining points of the event region.
  std::vector< point_with_id_t > _vertices;

  //! The event region converted into a polygon.
  boost::shared_ptr<Polygon> _polygon;

  std::vector<regulatory_element_ptr_t> _regulatory_elements;
};

typedef boost::shared_ptr< TrafficLight >  traffic_light_ptr_t;

} // namespace




#endif
