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

#include <liblanelet/LaneletMap.hpp>
#include <icl_core_config/Config.h>
#include <icl_core_logging/Logging.h>
#include <string>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <liblanelet/LaneletGraph.hpp>
#include <vector>
#include <boost/filesystem.hpp>

//! Represents a Node in an osm file.
struct OSMNode
{
  OSMNode(int id, double latitude, double longitude)
    : m_id(id),
      m_latitude(latitude),
      m_londitude(longitude) { }

  int id() const
  {
    return m_id;
  }

  double latitude() const
  {
    return m_latitude;
  }

  double londitude() const
  {
    return m_londitude;
  }

private:
  int m_id;
  double m_latitude;
  double m_londitude;
};


int next_way_id = -1;

//! Represents a way in an osm file.
struct OSMWay : public std::vector<OSMNode>
{
  typedef boost::shared_ptr<OSMWay> Ptr;

  OSMWay(int source_id, int target_id)
    : m_id(next_way_id--),
      m_source_vertex(source_id),
      m_target_vertex(target_id)
  {

  }


  int id() const
  {
    return m_id;
  }

  int source_vertex() const
  {
    return m_source_vertex;
  }

  int target_vertex() const
  {
    return m_target_vertex;
  }

private:
  int m_id;
  int m_source_vertex;
  int m_target_vertex;
};


//! Provides the function to write OSM Nodes and Ways to an osm file.
//!
struct OSMWriter
{
  OSMWriter(std::string file)
  {
    namespace fs = boost::filesystem;

    fs::path out(file);

    // if not absolute, make it absolute
    if (!out.is_absolute())
    {
      out = fs::current_path() / out;
    }

    fs::path parent_path = out.parent_path();
    if(!fs::exists(parent_path))
    {
      //If the folder structure for the output does not exist, create it!
      fs::create_directories(parent_path);
    }

    //open the stream.
    m_stream = boost::make_shared<std::ofstream>(out.c_str());
    *m_stream << std::setprecision(100);

    //write the header of the file.
    startFile();
  }


  ~OSMWriter()
  {
    //finsih the file when the writer is deleted.
    finishFile();
  }

  void writeOSMNode(OSMNode & node)
  {
    *m_stream << "    <node id='" << node.id() << "' visible='true' lat='" << node.latitude()
              << "' " << "lon='" << node.londitude() << "'>" << std::endl;
    *m_stream << "        <tag k='laneletgraphnode' v='true' />" << std::endl;
    *m_stream << "        <tag k='liblanelet_id' v='" << node.id() << "' />" << std::endl;
    *m_stream << "    </node>" << std::endl;
  }

  void writeOSMWay(OSMWay & way)
  {
    using std::endl;
    *m_stream << "    <way id='" << way.id() << "' visible='true' >" << endl;
    *m_stream << "      <tag k='laneletgraphedge' v='true' />" << endl;
    *m_stream << "      <nd ref='" << way.source_vertex() << "' />" << endl;
    *m_stream << "      <nd ref='" << way.target_vertex() << "' />" << endl;
    *m_stream << "    </way>" << endl;
  }

private:
  void startFile()
  {
    using std::endl;
    *m_stream << "<?xml version='1.0' encoding='UTF-8'?>" << endl;
    *m_stream << "  <osm version='0.6' upload='true' generator='JOSM'>" << endl;
  }

  void finishFile()
  {
    using std::endl;
    *m_stream << "</osm>" << endl;
  }

private:
  boost::shared_ptr<std::ofstream> m_stream;
};



int main(int argc, char ** argv)
{
  using std::endl;
  using std::cout;
  using namespace icl_core;
  using icl_core::config::getDefault;
  namespace fs = boost::filesystem;


  typedef LLet::Graph::vertex_descriptor Vertex;

  cout << "+--------------------------------------------------------------+" << endl;
  cout << "| Tool to convert a given lanelet-file to an ordinary osm file |" << endl;
  cout << "+---------------------------------------------------------------" << endl;

  cout << std::setprecision(100);


  config::addParameter(config::ConfigPositionalParameter("lanelet_map",
                                                         "/lanelet_map",
                                                         "The source lanelet file we want to read from.\n Defaults to \"/tmp/map.osm\".", true));

  config::addParameter(config::ConfigPositionalParameter("output_file",
                                                         "/output_file",
                                                         "The output osm file we want to write the graph ways to.\n Defaults to \"/tmp/graph.osm\".", true));


  //initialize the logging framework
  if(!icl_core::logging::initialize(argc, argv, true))
  {
    std::cerr << "The logging framework could not be loaded! Exit Program!" << std::endl;
    return -42;
  }

  std::string path_to_lanelet = getDefault<std::string>("/lanelet_map", "/tmp/map.osm");

  fs::path input(path_to_lanelet);
  if(!fs::exists(input))
  {
    std::cerr << "The given input file does not exist!" << std::endl
              << input.string() << std::endl << std::endl;
    return -1;
  }

  //The output folder.
  std::string output_path_str = getDefault<std::string>("/output_file", "/tmp/graph.osm");

  cout << "input-file:          " << input.string() << endl;
  cout << "output_file:         " << output_path_str << endl << endl;
  //Create a lanelet-map from the given file path.
  LLet::LaneletMap map(input.string(), true); // ignore_consistency_failures = true

  //Get the lanelet graph.
  LLet::Graph graph = map.graph();


  //Create the writer for the outpout file.
  OSMWriter writer(output_path_str);


  //Iterate over all vertices in the graph and write an osm node to the file.
  BGL_FORALL_VERTICES(v, graph, LLet::Graph)
  {
    LLet::lanelet_ptr_t lanelet = graph[v].lanelet;
    std::vector<LLet::point_with_id_t> center_points = lanelet->nodes(LLet::CENTER);

    const size_t size = center_points.size();
    size_t index = size / 2;

    LLet::point_with_id_t middle = center_points.at(index);

    using boost::get;
    //Write the node with the index and the long and lat value as a osm node.
    OSMNode node(lanelet->id(), get<LLet::LAT>(middle), get<LLet::LON>(middle));

    //Write the node to the file.
    writer.writeOSMNode(node);
  }


  BGL_FORALL_EDGES(e, graph, LLet::Graph)
  {
    Vertex source = boost::source(e, graph);
    Vertex target = boost::target(e, graph);

    int source_id = graph[source].lanelet->id();
    int target_id = graph[target].lanelet->id();


    OSMWay way(source_id, target_id);
    writer.writeOSMWay(way);
  }


  cout << "... done." << endl << endl;
  cout << "Load the output file into josm and use" << endl;
  cout << "  Preferences -> Display Settings -> OSM Data -> Draw Direction Arrows" << endl;
  cout << "to make the graph easily readable." << endl << endl;



  return 0;
}
