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

#include "Corridor.hpp"
#include "LaneletMap.hpp"

#include <set>

using namespace LLet;

LLet::Corridor::Corridor(const lanelet_ptr_t &start, const lanelet_ptr_t &end, const LaneletMap* map)
{
  if(!map)
  {
    return;
  }
  m_shortest_path = map->shortest_path(start, end);

  std::set<lanelet_ptr_t> querry_lanelets;
  m_all_lanelets.insert(m_shortest_path.begin(), m_shortest_path.end());
  querry_lanelets = m_all_lanelets;

  while(!querry_lanelets.empty())
  {
    lanelet_ptr_t current_lanelet = *(querry_lanelets.begin());
    querry_lanelets.erase(querry_lanelets.begin());

    std::set<lanelet_ptr_t> beside = map->beside_set(current_lanelet);

    std::pair<std::set<lanelet_ptr_t>::iterator,bool> ret;
    for(std::set<lanelet_ptr_t>::iterator iter = beside.begin(); iter != beside.end(); ++iter)
    {
        ret = m_all_lanelets.insert(*iter);
        if(ret.second)
        {
          querry_lanelets.insert(*iter);
        }
    }
  }

  //get ends in relation

  querry_lanelets = m_all_lanelets;

  for(std::set<lanelet_ptr_t>::iterator iter = m_all_lanelets.begin(); iter != m_all_lanelets.end(); ++iter)
  {
    std::set<lanelet_ptr_t> following = map->following_set(*iter, true);
    for(std::set<lanelet_ptr_t>::iterator follower = following.begin(); follower != following.end(); ++follower)
    {
      if(querry_lanelets.find(*follower) != querry_lanelets.end())
      {
        m_ends_in[*iter] = *follower;
        querry_lanelets.erase(*follower);
        break;
      }
    }
  }


  //precalculate lanes
  for(std::size_t lane_number = 0; querry_lanelets.size() > 0; ++lane_number)
  {
    lanelet_ptr_t current = *(querry_lanelets.begin());
    querry_lanelets.erase(querry_lanelets.begin());

    //construct one lane
    while(current)
    {
      m_is_in_lane[current] = lane_number;
      m_lanes[lane_number].push_back(current);

      current = m_ends_in[current];
    }
  }
}

const std::vector<lanelet_ptr_t>& Corridor::getShortestPath() const
{
  return m_shortest_path;
}

std::vector<lanelet_ptr_t> Corridor::getLaneOnCorridor(lanelet_ptr_t lanelet_on_lane) const
{
  std::map<lanelet_ptr_t, std::size_t>::const_iterator found = m_is_in_lane.find(lanelet_on_lane);
  if(found != m_is_in_lane.end())
  {
    // TODO C++11 : change this to m_lanes.at(found->second);
    return m_lanes.find(found->second)->second;
  }
  return std::vector<lanelet_ptr_t>();
}

const std::set<lanelet_ptr_t>& Corridor::getAllLanelets() const
{
  return m_all_lanelets;
}
