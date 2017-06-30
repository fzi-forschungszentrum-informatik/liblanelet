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

#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/optional.hpp>

#include <boost/foreach.hpp>

#include "Lanelet.hpp"
#include "LocalGeographicCS.hpp"
#include <vector>
#include <vector>
#include <deque>
#include <queue>
#include <map>

#include <liblanelet/RTree.h>

namespace LLet
{

struct EdgeInfo
{
    double routing_cost;
};

struct VertexInfo
{
    lanelet_ptr_t lanelet;
};

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo, EdgeInfo > Graph;

typedef Graph::vertex_descriptor node_t;
typedef Graph::edge_descriptor arc_t;

template < typename G >
class distance_heuristic : public boost::astar_heuristic< G, double>
{
public:

    distance_heuristic(G& graph, typename G::vertex_descriptor target) : graph(graph), target(target)
    {}
    double operator()(typename G::vertex_descriptor u)
    {
        return 0;
    }

private:
    G& graph;
    typename G::vertex_descriptor target;
};

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
        if(u == m_goal)
            throw found_goal();
    }
private:
    Vertex m_goal;
};

template< typename G >
struct Dijkstra
{
    typedef typename G::vertex_descriptor vertex_t;
    G& graph;
    vertex_t source;

    std::vector< vertex_t > _predecessors;
    std::vector< double > _distances;

    void abort_if_vertex_invalid( vertex_t v )
    {
        if( v < 0 || v >= boost::num_vertices(graph) )
        {
            std::cerr << "invalid vertices in Dijkstra." << std::endl;
            std::cerr << v << " (" << boost::num_vertices(graph) << ")" << std::endl;
            abort();
        }
    }

    Dijkstra( G& graph, vertex_t source ) : graph(graph), source(source)
    {
        using namespace boost;

        abort_if_vertex_invalid( source );
        size_t num_vertices = boost::num_vertices( graph );
        this->_predecessors.resize( num_vertices );
        this->_distances.resize( num_vertices );
        std::fill( _distances.begin(), _distances.end(), std::numeric_limits<double>::max() );

        boost::dijkstra_shortest_paths
                (graph, source,
                 predecessor_map(_predecessors.data()).distance_map(_distances.data()).
                 weight_map(get(&EdgeInfo::routing_cost, graph)));
    }

    std::deque< vertex_t > shortest_path( vertex_t target )
    {
        abort_if_vertex_invalid( target );
        const static std::deque< vertex_t > EMPTY_LIST;
        if( !reached(target) )
            return EMPTY_LIST;

        std::deque< vertex_t > sp;
        for( vertex_t v = target; v != source ; v = _predecessors[v] )
        {
            sp.push_front( v );
        }

        sp.push_front( source );

        assert((sp.empty() || ((sp.front() == source) && (sp.back() == target)))
               && "Dijkstra returns either empty list or path from source to target.");

        return sp;
    }

    bool reached( vertex_t target )
    {
        static const double BIG_NUM = 1e9;
        return _distances[target] < BIG_NUM;
    }
};

template<typename G>
boost::optional< std::vector< typename G::vertex_descriptor > >
dijkstra_shortest_path( G& g, typename G::vertex_descriptor from, typename G::vertex_descriptor to )
{
    boost::optional< std::vector< typename G::vertex_descriptor > > result;
     Dijkstra< G > dij( g, from );

     if(!dij.reached(to))
         return result;

     std::deque< typename G::vertex_descriptor > sp = dij.shortest_path( to );

     std::vector< typename G::vertex_descriptor > my_sp( sp.size() );
     std::copy(sp.begin(), sp.end(), my_sp.begin());

     assert(((my_sp.empty() && sp.empty()) || ((my_sp.front() == sp.front()) && (my_sp.back() == sp.back())))
            && "deque and vector are equal."  );

     result = my_sp;
     return result;
}

}

