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
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2014-11-4
*
*/
//----------------------------------------------------------------------

#include <string>
#include <fstream>
#include <liblanelet/LineStrip.hpp>
#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/Corridor.hpp>

using namespace LLet;

int main(int argc, char *argv[])
{
  std::string filename = "";
  if (argc > 1)
  {
    filename = std::string(argv[1]);
  }
  else
  {
    std::cerr << "usage: " << argv[0] << " <lanelet-filename>" << std::endl;
    return -1;
  }

  lanelet_map_ptr_t map(new LaneletMap(filename));

  assert(map->lanelets().empty() == false);

  lanelet_ptr_t start = map->lanelet_by_id(-500);
  lanelet_ptr_t end = map->lanelet_by_id(-520);

  corridor_ptr_t corridor = map->corridor(start, end);

  std::cout << "All Lanelets" << std::endl;

  for(lanelet_ptr_t lanelet: corridor->getAllLanelets())
  {
    std::cout << lanelet << std::endl;
  }

  std::cout << "Lane of starting lanelet" << std::endl;

  for(lanelet_ptr_t lanelet: corridor->getLaneOnCorridor(start))
  {
    std::cout << lanelet << std::endl;
  }



}
