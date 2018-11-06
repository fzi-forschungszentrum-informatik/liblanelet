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
 * \author  Florian Kuhnt <kuhnt@fzi.de>
 * \author  Tobias Fleck <tfleck@fzi.de>
 * \date    2016-06-03
 *
 */
//----------------------------------------------------------------------

#include <iostream>

#include <icl_core_config/Config.h>
#include <icl_core_logging/Logging.h>

#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/LaneletGraph.hpp>

#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

#include <map>
#include <boost/property_map/property_map.hpp>

#include <vector>

#include <utility>

using namespace std;

int main(int argc, char ** argv)
{
  using namespace icl_core;

  cout << "+-----------------------------------------------------------------+" << endl;
  cout << "| Tool to convert a given lanelet-file to a graphml file for yed. |" << endl;
  cout << "+-----------------------------------------------------------------+" << endl;

  cout << "THIS IS NOT COMPLETELY IMPLEMENTED YET. PLEASE COMPLETE IT BEFORE USING IT!" << endl;


  //define parameter
  config::addParameter(config::ConfigPositionalParameter("lanelet_map",
                                                         "/lanelet_map",
                                                         "The source lanelet file we want to read from.\n Defaults to \"/tmp/map.osm\".", true));

  config::addParameter(config::ConfigPositionalParameter("output_file",
                                                         "/output_file",
                                                         "The output graphviz file we want to write the graph to.\n Defaults to \"/tmp/graph.graphml\".", true));


  //Initialize logger.
  icl_core::logging::initialize(argc, argv);

  //Get the parameter.
  std::string path_to_lanelet_map = config::getDefault<std::string>("/lanelet_map", "/tmp/map.osm");
  std::string path_to_output = config::getDefault<std::string>("/output_file", "/tmp/graph.graphml");

  std::cout << "Reading Lanelet from file:" << std::endl;
  std::cout << path_to_lanelet_map << std::endl;
  std::cout << "Converting Lanelet..." << std::endl;


  //Load the lanelet-map.
  LLet::LaneletMap map(path_to_lanelet_map, true); // ignore_consistency_failures = true

  std::cout << "The graph contains " << map.lanelets().size() << " lanelets." << std::endl;
  LLet::Graph graph = map.graph();

  //out-stream for the file to write.
  std::ofstream os;
  os.open(path_to_output);

  if(!os.is_open())
  {
    std::cout << "Failed to write file:" << std::endl
              << path_to_output << std::endl
              << "Finishing program..." << std::endl;
    os.close();
    return -42;
  }


  //Create a property map for the latitude and longitude of the vertices.
  //Then store it in dynamic properties. then call the writing algorithm
  //in boost for the graphviz file.

  std::map<LLet::Graph::vertex_descriptor, double> m_vertex_latitudes;
  std::map<LLet::Graph::vertex_descriptor, double> m_vertex_longitudes;
  std::map<LLet::Graph::vertex_descriptor, int> m_vertex_ids;

  BGL_FORALL_VERTICES(v, graph, LLet::Graph)
  {
    LLet::lanelet_ptr_t lanelet = graph[v].lanelet;

    //get the points of the lanelet middle stripe.
    std::vector<LLet::point_with_id_t> center_points = lanelet->nodes(LLet::CENTER);
    const size_t size = center_points.size();
    size_t index = size / 2;
    LLet::point_with_id_t middle = center_points.at(index);

    double lat = boost::get<LLet::LAT>(middle);
    double lon = boost::get<LLet::LON>(middle);
    m_vertex_latitudes.insert(std::pair<LLet::Graph::vertex_descriptor, double>(v, lat));
    m_vertex_longitudes.insert(std::pair<LLet::Graph::vertex_descriptor, double>(v, lon));
    m_vertex_ids.insert(std::pair<LLet::Graph::vertex_descriptor, int>(v, lanelet->id()));
  }


  boost::dynamic_properties dp;

  boost::associative_property_map< std::map<LLet::Graph::vertex_descriptor, double> >
      longitudes(m_vertex_longitudes);
  boost::associative_property_map< std::map<LLet::Graph::vertex_descriptor, double> >
      latitudes(m_vertex_latitudes);
  boost::associative_property_map< std::map<LLet::Graph::vertex_descriptor, int> >
      ids(m_vertex_ids);


  dp.property("longitude", longitudes);
  dp.property("latitude", latitudes);
  dp.property("id", ids);

  boost::write_graphml(os, graph, dp, true);


  std::cout << "Writing graph to file" << std::endl;
  std::cout << path_to_output << std::endl;
  std::cout << "Writing Successful!" << std::endl << std::endl;
  //close the filestream.
  os.close();


  cout << "Load the output file into yed and choose a layout in the menu " << endl;
  cout << "to make the graph structure easily readable." << endl << endl;

  return 0;
}
