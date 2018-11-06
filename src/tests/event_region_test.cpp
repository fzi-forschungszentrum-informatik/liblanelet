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
 * \author  Tim Pollert <pollert@fzi.de>
 *
 */
//----------------------------------------------------------------------

#include <liblanelet/llet_xml.hpp>
#include <liblanelet/Lanelet.hpp>
#include <liblanelet/EventRegion.hpp>
#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/RegulatoryElement.hpp>
#include <liblanelet/prettyprint.hpp>

#include <stdio.h>
#include <string>

using namespace LLet;

int main(int argc, char **argv)
{
  std::cout << "=== Start event region parser test ===" << std::endl;

  /* ----------------------------------------------------
   *  Read in test file from /tmp/
   * ---------------------------------------------------- */
  std::string filepath = "/tmp/test_lanelet.osm";
  std::cout << "Read file: " << filepath << std::endl;

  /* ----------------------------------------------------
   *  Parsing result will be stored in event regions.
   * ---------------------------------------------------- */
  std::cout << "Parse osm file." << std::endl;

  std::cout << "The test file include two event regions and one with a flaw!" << std::endl;
  std::cout << "This flaw will cause a parsing error!" << std::endl;

  std::vector< event_region_ptr_t > event_regions = parse_xml_event_regions(filepath);
  std::cout << std::endl;

  /* ----------------------------------------------------
   *  Parsing result will be stored in event regions.
   * ---------------------------------------------------- */
  std::cout << " == Results == " << std::endl;
  std::cout << "" << std::endl;

  std::cout << "Event Regions" << std::endl;
  for(size_t index = 0; index < event_regions.size(); index++)
  {
    std::cout << "Event Region with id " << event_regions[index]->getId() << " with size of " << event_regions[index]->getVertices().size();
    std::cout << " and type: " << event_regions[index]->getType() <<std::endl;
  }

  return 1;
}
