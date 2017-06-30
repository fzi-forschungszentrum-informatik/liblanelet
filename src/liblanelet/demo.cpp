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
 * \author  Philipp Bender <philipp.bender@fzi.de>
 * \date    2014-01-01
 *
 */
//----------------------------------------------------------------------

#include <iostream>

#include <liblanelet/llet_xml.hpp>
#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/BoundingBox.hpp>

#include <boost/foreach.hpp>

#include <vector>
#include <boost/tuple/tuple.hpp>
#include <memory>

using namespace LLet;

int main (int argc, char* argv[]) {
    if (argc != 2)
    {
      std::cerr << "Usage: " << argv[0] << " file.osm" << std::endl;
      return 0;
    }

    std::string source = argv[1];
    LaneletMap the_map( source );

    BoundingBox world( boost::make_tuple(-180.0, -180.0, 180.0, 180.0) );
    std::vector<lanelet_ptr_t> query_result = the_map.query(world);

    // print all attributes of each lanelet
    BOOST_FOREACH( const lanelet_ptr_t& lanelet, query_result )
    {
        const AttributeMap& attributes = lanelet->attributes();
        BOOST_FOREACH( const AttributeMap::value_type& a, attributes )
        {
            std::cout << a.first << ": " << a.second.as_string() << std::endl;
        }
    }

    // exploiting the lanelet graph directly
    const Graph& G = the_map.graph();
    BOOST_FOREACH(const arc_t& edge, boost::edges(G))
    {
      node_t src_vtx  = boost::source(edge, G);
      node_t dest_vtx = boost::target(edge, G);
      std::cout << G[src_vtx].lanelet << " --> " << G[dest_vtx].lanelet << std::endl;
    }
}
