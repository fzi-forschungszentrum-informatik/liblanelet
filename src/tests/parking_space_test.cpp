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


#include <liblanelet/llet_xml.hpp>
#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/RegulatoryElement.hpp>
#include <liblanelet/prettyprint.hpp>

#include <stdio.h>
#include <string>

using namespace LLet;

int main(int argc, char **argv)
{
  std::cout << "=== Start parking space parser test ===" << std::endl;

  /* ----------------------------------------------------
   *  Read in test file from /tmp/
   * ---------------------------------------------------- */
  std::string filepath = "/tmp/test.osm";
  //std::string filepath = "/tmp/campus_ost_lanelets_test.osm";
  std::cout << "Read file: " << filepath << std::endl;


  /* ----------------------------------------------------
   *  Pars file and get lanelets and parking spaces.
   * ---------------------------------------------------- */
  std::cout << "Parse lanelets and parking spaces from file." << std::endl;
  std::vector< lanelet_ptr_t > lanelets = parse_xml(filepath);
  std::vector< parking_space_ptr_t > parking_spaces = parse_xml_parking_spaces(filepath);


  /* ----------------------------------------------------
   * Print results
   * ---------------------------------------------------- */
  std::cout << " == Results == " << std::endl << std::endl;

  std::cout << "Number of lanelets: " << lanelets.size() << std::endl;

  /*
  std::cout << "Lanelets:" << std::endl;
  for(size_t index = 0; index < lanelets.size(); index++)
  {
    std::cout << "Lanelet with id: " << lanelets[index]->id() << std::endl;
  }
  */

  std::cout << "Parking Spaces" << std::endl;
  for(size_t index = 0; index < parking_spaces.size(); index++)
  {
    std::cout << parking_spaces[index]->getId() << " with related lanelets: ";
    for(size_t j = 0; j < parking_spaces[index]->getRelatedLanelets().size(); j++)
    {
      std::cout << parking_spaces[index]->getRelatedLanelets().at(j) << ", ";
    }
    std::cout << " and parking direction: " << parking_spaces[index]->getParkingDirection() <<std::endl;
  }


  /* ----------------------------------------------------
   * Start shortest path test for lanelet to parking space
   * ---------------------------------------------------- */

  std::cout << "=== Start shorest path test (lanelets to parking space)===" << std::endl;


  /* ----------------------------------------------------
   * Generate Lanelet Map
   * ---------------------------------------------------- */

  LLet::LaneletMap lanelet_map(filepath);
  std::cout << "The lanelet map contains " << lanelet_map.lanelets().size() << " lanelets." << std::endl;
  std::cout << "Graph size: " <<  boost::num_vertices(lanelet_map.graph())<< std::endl;

  int32_t start_lanelet_id = -9785;
  std::cout << "Note: A fixed lanelet with id " << start_lanelet_id <<" is used for testing!" << std::endl;
  lanelet_ptr_t start = lanelet_map.lanelet_by_id(start_lanelet_id);

  if (parking_spaces.size() > 0)
  {
    parking_space_ptr_t end = parking_spaces.at(0);
    std::cout << "Note: A fixed parking space with id " << end->getId() << " is used for testing!" << std::endl;
    std::cout << "      All related lanelets of the parking space will be used as a set of destinations." << std::endl;

    std::vector< lanelet_ptr_t > lanelet_shortest_path = lanelet_map.shortest_path(start, end);

    if (lanelet_shortest_path.size() > 0)
    {
      std::cout << "Found a path with size: " << lanelet_shortest_path.size() << std::endl;
    }
    else
    {
      std::cout << "No path found" << std::endl;
    }
  }

  return 1;
}

