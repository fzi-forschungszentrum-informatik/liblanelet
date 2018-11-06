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
 * \author  Andreas Fleig <fleig@fzi.de>
 * \date    2016-09-20
 *
 */
//----------------------------------------------------------------------

#include <iostream>

#include <icl_core_config/Config.h>

#include <liblanelet/llet_xml.hpp>
#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/BoundingBox.hpp>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/tuple/tuple.hpp>

#include <vector>
#include <memory>

using namespace LLet;
using namespace std;

int main (int argc, char* argv[])
{
  using namespace icl_core;
  using icl_core::config::getDefault;
  namespace fs = boost::filesystem;

  cout << "+-------------------------------+" << endl;
  cout << "| Tool to check osm-lanelet-map |" << endl;
  cout << "+-------------------------------+" << endl;

  cout << std::setprecision(100);

  config::addParameter(config::ConfigPositionalParameter("lanelet_map",
                                                         "/lanelet_map",
                                                         "The source lanelet file we want to read from.\n Defaults to \"/tmp/map.osm\".", true));

  config::addParameter(config::ConfigPositionalParameter("output_file",
                                                         "/output_file",
                                                         "The output osm file we want to write to.\n Defaults to \"/tmp/optimized_map.osm\".", true));

  config::addParameter(config::ConfigParameter("ignore_consistency_failures", "ignore_failures",
                                               "/ignore_consistency_failures",
                                               "Ignore occuring consistency failures."));

  //initialize the logging framework
  if(!icl_core::logging::initialize(argc, argv, true))
  {
    std::cerr << "The logging framework could not be loaded! Exit Program!" << std::endl;
    return -42;
  }

  std::string source = getDefault<std::string>("/lanelet_map", "/tmp/map.osm");

  fs::path input(source);
  if(!fs::exists(input))
  {
    std::cerr << "The given input file does not exist!" << std::endl
              << input.string() << std::endl << std::endl;
    return -1;
  }

  //The output folder.
  std::string output_path_str = getDefault<std::string>("/output_file", "/tmp/optimized_map.osm");

  cout << "input-file:          " << input.string() << endl;
  cout << "output_file:         " << output_path_str << endl << endl;

  LaneletMap the_map( source, config::getDefault<bool>("/ignore_consistency_failures", false) );

  BoundingBox world( boost::make_tuple(-180.0, -180.0, 180.0, 180.0) );
  std::vector<lanelet_ptr_t> query_result = the_map.query(world);

  // print all attributes of each lanelet
  /*
  BOOST_FOREACH( const lanelet_ptr_t& lanelet, query_result )
  {
      const AttributeMap& attributes = lanelet->attributes();
      BOOST_FOREACH( const AttributeMap::value_type& a, attributes )
      {
          std::cout << a.first << ": " << a.second.as_string() << std::endl;
      }
  }*/

  // exploiting the lanelet graph directly
  /*const Graph& G = the_map.graph();
  BOOST_FOREACH(const arc_t& edge, boost::edges(G))
  {
    node_t src_vtx  = boost::source(edge, G);
    node_t dest_vtx = boost::target(edge, G);
    std::cout << G[src_vtx].lanelet << " --> " << G[dest_vtx].lanelet << std::endl;
  }*/

  // write out corrected LaneletMap as osm file:
  std::cout << std::endl << "writing out improved osm to " << output_path_str << std::endl;
  std::cout << "ouput-file can be set via parameter" << std::endl;
  TiXmlDocument doc = the_map.toXml();
  doc.SaveFile(output_path_str);

  cout << "... done." << endl << endl;
}
