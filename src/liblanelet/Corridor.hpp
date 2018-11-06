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
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2014-11-4
*
*/
//----------------------------------------------------------------------
#ifndef CORRIDOR_H
#define CORRIDOR_H

#include "LaneletFwd.hpp"
#include "ImportExport.h"

#include <map>
#include <vector>
#include <set>

namespace LLet
{

class LaneletMap;

/*!
 * \brief The Corridor class represents the drivable area of a vehicle on its route.
 * This is a collection of lanelets the vehicle can drive on to follow a certain route to a destination.
 */
class LANELET_IMPORT_EXPORT Corridor
{
friend class LaneletMap;

public:
  /*!
   * \brief getShortestPath
   * \return the shortest path between start and end.
   * \see LaneletMap::shortest_path
   */
  const std::vector<lanelet_ptr_t>& getShortestPath() const;

  /*!
   * \brief getLaneOnCorridor teturns the lane on the corridor where the given lanelet is on
   * \param lanelet_on_lane the lanelet that defines the lane
   * \param index returns the index of the lanelets on the lane
   * \return a vector containing an ordered list of all lanelets of the quaried lane.
   */
  std::vector<lanelet_ptr_t> getLaneOnCorridor(lanelet_ptr_t lanelet_on_lane) const;

  /*!
   * \brief getAllLanelets returns all lanelets of the corridor
   * \return a set of all lenelets of the defined corridor
   */
  const std::set<lanelet_ptr_t>& getAllLanelets() const;

private:

  /*!
   * \brief Corridor
   * \param start the start lanelet of the corridor. Often the lanelet the vehicle is on.
   * \param end the end or target lanelet of the corridor.
   */
  Corridor(const lanelet_ptr_t& start, const lanelet_ptr_t& end, const LLet::LaneletMap *map);


  std::map<lanelet_ptr_t, lanelet_ptr_t> m_ends_in;
  std::map<lanelet_ptr_t, std::size_t> m_is_in_lane;
  std::map<std::size_t,std::vector<lanelet_ptr_t> > m_lanes;

  std::vector< lanelet_ptr_t >m_shortest_path;
  std::set<lanelet_ptr_t> m_all_lanelets;
};

typedef boost::shared_ptr< Corridor > corridor_ptr_t;

}

#endif // CORRIDOR_H
