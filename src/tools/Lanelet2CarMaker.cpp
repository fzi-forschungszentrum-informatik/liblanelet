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
 * \author  Marc Zofka <zofka@fzi.de>
 * \date    2014-01-01
 *
 */
//----------------------------------------------------------------------


#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <liblanelet/LaneletMap.hpp>

using namespace std;
using namespace LLet;

void write_file_header(fstream& ipgroad, std::string& description)
{
  ipgroad << ": x y z q wl wr ml mr" << endl;
  ipgroad << "#" << endl;
  ipgroad << "#" << " " << description << endl;
  ipgroad << "#" << endl;
  ipgroad << "# Created with Lanelet2CarMaker Converter" << endl;
  ipgroad << "# (c) 2014 FZI Forschungszentrum Informatik Karlsruhe" << endl;
  ipgroad << "#" << endl;
  ipgroad << "#x     y           z   q  wl wr  ml  mr" << endl;
}

int main(int argc, char *argv[])
{
  string lanelet_filename = "lanelet.osm";
  string ipgroad_filename = "sample.ipgroad";
  string description      = "Road_Model";
  int32_t lanelet_id      =    0;
  double ref_lat          = -1.0;
  double ref_lon          = -1.0;
  double lane_width       =  0.0;
  double lane_margin      =  0.0;
  double interpol_dist    =  0.0;

  if (argc > 9)
  {
    lanelet_filename = std::string(argv[1]);
    ipgroad_filename = std::string(argv[2]);
    description      = std::string(argv[3]);
    lanelet_id       = atoi(argv[4]);
    ref_lat     = atof(argv[5]);
    ref_lon     = atof(argv[6]);
    lane_width  = atof(argv[7]);
    lane_margin = atof(argv[8]);
    interpol_dist = atof(argv[9]);
  }
  else
  {
    std::cerr << "usage: " << argv[0] << " <lanelet-filename> <ipgroad-filename> <file-description> <lanelet-id> <reference_lat> <reference_lon> <lane_width> <lane_margin> <interpol_dist>" << std::endl;
    std::cerr << "example: " << "lanelet_to_CarMaker A0_lanelets_campus_east.osm A0_Campus_East.ipgroad A0_Demo -304 49.0231074744 8.43137100415  3.5 1.0 1.0" << std::endl;
    return -1;
  }

  // open file to write ipgroad
  fstream ipgroad;
  ipgroad.open(ipgroad_filename, ios_base::in|ios_base::out|ios_base::trunc);

  if(ipgroad)
  {
    std::cout << "Opened file: " << argv[2] << endl;
  }
  else
  {
    std::cerr << "Could not open file to write IPG Road."<< std::endl;
  }

  write_file_header(ipgroad, description);

  // Create reference point
  point_with_id_t reference_point;
  boost::get<LAT>(reference_point) = ref_lat;
  boost::get<LON>(reference_point) = ref_lon;

  // Obtain start_lanelet
  std::cout << "Opening Lanelet Map and start with lanelet "<< lanelet_id << std::endl;
  lanelet_map_ptr_t map(new LaneletMap(lanelet_filename));
  lanelet_ptr_t start_lanelet = map->lanelet_by_id(lanelet_id);

  bool first_lanelet = true;

  lanelet_ptr_t current_lanelet = start_lanelet;
  do
  {
    // Take points of left strip and interpolate
    strip_ptr_t left_strip = boost::get<RIGHT>(current_lanelet->bounds());
    left_strip->interpolate_spline(interpol_dist);

    std::vector<point_with_id_t> strip_points = left_strip->pts();

    point_xy_t p;
    for(std::size_t i = 0; i < strip_points.size(); ++i)
    {
      //avoid taking one point twice because of start and end point
      if( i==0 && !first_lanelet)
      {
        continue;
      }
      else if(i==0 && first_lanelet)
      {
        first_lanelet = false;
      }

      p = vec(reference_point, strip_points[i]);

      double x = boost::get<X>(p);
      double y = boost::get<Y>(p);

      // line
      ipgroad << -y << " ";
      ipgroad << -x << " ";
      ipgroad << "0.0" << " "; // z
      ipgroad << "0.0" << " "; // slope of road q
      ipgroad << lane_width << " "; // width left
      ipgroad << lane_width << " "; // width right
      ipgroad << lane_margin << " "; // margin left
      ipgroad << lane_margin << " "; // margin right
      ipgroad << std::endl;
      //std::cout << "[X,Y] = " << "[" << x << " " << y << "]"<< std::endl;
      std::cout << "ID " << current_lanelet->id() <<" [X,Y] = " << "[" << x << " " << y << "]"<< std::endl;
    }

    // Obtain following lanelet
    std::set<lanelet_ptr_t> following_lanelets = map->following_set(current_lanelet, true);
    current_lanelet = *following_lanelets.begin();
    std::cout << "New Lanelet with id " << current_lanelet->id() << std::endl;

  } while(start_lanelet->id() != current_lanelet->id());

  // close file
  ipgroad.close();
  return 0;
}
