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
 * \author  Philipp Bender <philipp.bender@fzi.de>
 * \date    2014-01-01
 *
 */
//----------------------------------------------------------------------

#include "LaneletMap.hpp"
#include "LineStrip.hpp"
#include "llet_xml.hpp"
#include "Polygon.hpp"
#include <iostream>
#include <algorithm>
#include <sstream>
#include <cassert>
#include "LaneletGraph.hpp"
#include "prettyprint.hpp"


using namespace LLet;

LaneletMap::LaneletMap(const std::vector<lanelet_ptr_t> &lanelets) : _lanelets(lanelets)
{
    init();
}

LaneletMap::LaneletMap(const std::string &filename, const bool ignore_consistency_failures)
  : _lanelets(LLet::parse_xml(filename, ignore_consistency_failures)),
    _parking_spaces(LLet::parse_xml_parking_spaces(filename, ignore_consistency_failures)),
    _event_regions(LLet::parse_xml_event_regions(filename, ignore_consistency_failures)),
    _traffic_lights(LLet::parse_xml_traffic_lights(filename, ignore_consistency_failures)),
    _minimum_node_id(LLet::get_minimum_node_id(filename, ignore_consistency_failures))
{
    init();
}

std::vector<lanelet_ptr_t> LaneletMap::query(const BoundingBox &box) const
{
    return _lanelet_tree.query(box);
}

std::vector<lanelet_ptr_t> LaneletMap::shortest_path(const lanelet_ptr_t &start, const lanelet_ptr_t &dest) const
{
    int64_t start_index = vertex_id_by_lanelet(start);
    int64_t dest_index = vertex_id_by_lanelet(dest);


    boost::optional< std::vector<size_t> > sp = dijkstra_shortest_path(graph(), start_index, dest_index);

    if( !sp || (*sp).empty() )
    {
      return std::vector< lanelet_ptr_t >();
    }

    std::vector< lanelet_ptr_t > sp_ll(sp->size());


    // this replaces: std::transform(sp->cbegin(), sp->cend(), sp_ll.begin(), [this](int64_t index){ return graph()[index].lanelet;});
    size_t i = 0;
    BOOST_FOREACH(int64_t index, *sp)
    {
      sp_ll[i++] = graph()[index].lanelet;
    }

    return sp_ll;
}

LaneletMap::lanelet_ptr_vector LaneletMap::shortest_paths(const lanelet_ptr_vector &starts,
                                                                         const lanelet_ptr_vector &destinations ) const
{
  std::vector<lanelet_ptr_vector> all_paths;
  for (size_t i=0; i < starts.size(); ++i)
  {
    std::vector<lanelet_ptr_vector> paths_starting_at_i;
    paths_starting_at_i = shortest_paths(starts[i], destinations);
    std::cout << "paths_at_i:" << i << " : " << starts[i]->id() << " " << paths_starting_at_i.size() << std::endl;
    all_paths.insert(all_paths.end(), paths_starting_at_i.begin(), paths_starting_at_i.end()); 
  }
  std::cout << "all_paths:" << all_paths.size() << std::endl;
  return shortest_path(all_paths);
}

std::vector< LaneletMap::lanelet_ptr_vector > LaneletMap::shortest_paths(const lanelet_ptr_t &start,
                                                                         const lanelet_ptr_vector &destinations ) const
{
  std::vector< lanelet_ptr_vector > result;

  for (size_t index = 0; index < destinations.size(); ++index)
  {
    lanelet_ptr_vector path = shortest_path(start, destinations[index]);
    if (path.size() > 0)
    {
      result.push_back(path);
    }
  }

  return result;
}

LaneletMap::lanelet_ptr_vector LaneletMap::shortest_path(const std::vector< lanelet_ptr_vector >& paths ) const
{
  lanelet_ptr_vector search_result;
  double shortest_path_length = std::numeric_limits<double>::max();

  for (size_t path_index = 0; path_index < paths.size(); ++path_index)
  {
    lanelet_ptr_vector path = paths[path_index];
    double path_length = 0;
    for (size_t lanelet_index = 0; lanelet_index < path.size(); ++lanelet_index)
    {
      path_length += path[lanelet_index]->length(CENTER);
    }

    if (path_length < shortest_path_length)
    {
      search_result = path;
      shortest_path_length = path_length;
    }
  }

  return search_result;
}

LaneletMap::lanelet_ptr_vector LaneletMap::shortest_path( const lanelet_ptr_t& start, const parking_space_ptr_t& destination ) const
{
  // Convert the related lanelets to a lanelet ptr vector
  // Related lanelets are potential destinations.
  const size_t number_of_destinations = destination->getRelatedLanelets().size();
  if (number_of_destinations == 0)
  {
    std::cout << "LaneletMap::shortest_path: Warning parking space related lanelets is empty! Cannot find valid route." << std::endl;
  }
  lanelet_ptr_vector destinations;

  for (size_t index = 0; index < number_of_destinations; ++index)
  {
    destinations.push_back(lanelet_by_id(destination->getRelatedLanelets()[index]));
  }

  std::vector< lanelet_ptr_vector > paths = shortest_paths(start, destinations);

  return shortest_path(paths);
}

std::set<lanelet_ptr_t> LaneletMap::reachable_set(const lanelet_ptr_t& start, double max_distance) const
{
  std::set<lanelet_ptr_t> result;

  std::priority_queue<ReachableSetInfo> queue;
  std::map<int64_t,ReachableSetInfo> nodes;
  queue.push(ReachableSetInfo(vertex_id_by_lanelet(start), 0));
  Graph::out_edge_iterator iter, end;

  typedef boost::adj_list_edge_property_map<boost::bidirectional_tag, double, const double&, size_t, const LLet::EdgeInfo, double LLet::EdgeInfo::*> weights_map_t;
  weights_map_t weights = boost::get(&EdgeInfo::routing_cost, _graph);
  while (!queue.empty())
  {
    // Examine nodes with lowest distance first
    ReachableSetInfo const current = queue.top();
    queue.pop();
    if (nodes.find(current.index) == nodes.end())
    {
      nodes[current.index] = current;
    }
    for (boost::tie(iter, end) = out_edges(current.index, _graph); iter != end; ++iter)
    {
      // Update target nodes, if necessary
      double const weight = boost::get(weights, *iter);
      assert(weight > 0 && "non-positive weight encountered, termination condition violated");
      lanelet_ptr_t const lanelet = _graph[boost::target(*iter, _graph)].lanelet;
      int64_t idx = vertex_id_by_lanelet(lanelet);
      if (nodes.find(idx) == nodes.end())
      {
	nodes[idx] = ReachableSetInfo(idx, current.total_distance + weight);
      }

      bool updated = false;
      ReachableSetInfo target = nodes[idx];
      if (current.total_distance + weight <= target.total_distance)
      {
	// New shortest way found: Update target node
	target.total_distance = current.total_distance + weight;
	nodes[idx] = target;
	updated = true;
      }

      if (target.total_distance < max_distance)
      {
	// Node is reachable; queue examining its targets
	result.insert(lanelet);
	if (updated) // otherwise already queued/visited
	{
	  queue.push(target);
	}
      }
    }
  }

  return result;
}

std::set<lanelet_ptr_t> LaneletMap::following_set(const lanelet_ptr_t& start, bool filter_out_lane_changes) const
{
  std::set<lanelet_ptr_t> result;
  Graph::out_edge_iterator iter, end;
  int64_t current_id = vertex_id_by_lanelet(start);
  for (boost::tie(iter, end) = out_edges(current_id, _graph); iter != end; ++iter)
  {
    lanelet_ptr_t lanelet = _graph[boost::target(*iter, _graph)].lanelet;
    result.insert(lanelet);
  }
  if (filter_out_lane_changes)
  {
    for (std::set<lanelet_ptr_t>::iterator it=result.begin(); it!=result.end(); /* */)
    {
      if ((start->fits_beside(*it, LLet::LEFT)) || (start->fits_beside(*it, LLet::RIGHT)))
      {
        result.erase(it++); //implicitly save a copy of it
      }
      else
      {
        ++it;
      }
    }
  }
  return result;
}

std::set<lanelet_ptr_t> LaneletMap::previous_set(const lanelet_ptr_t& start, bool filter_out_lane_changes) const
{
  std::set<lanelet_ptr_t> result;
  Graph::in_edge_iterator iter, end;
  int64_t current_id = vertex_id_by_lanelet(start);
  for (boost::tie(iter, end) = in_edges(current_id, _graph); iter != end; ++iter)
  {
    lanelet_ptr_t lanelet = _graph[boost::source(*iter, _graph)].lanelet;
    result.insert(lanelet);
  }
  if (filter_out_lane_changes)
  {
    for (std::set<lanelet_ptr_t>::iterator it=result.begin(); it!=result.end(); /* */)
    {
      if ((start->fits_beside(*it, LLet::LEFT)) || (start->fits_beside(*it, LLet::RIGHT)))
      {
        result.erase(it++); //implicitly save a copy of it
      }
      else
      {
        ++it;
      }
    }
  }
  return result;
}

std::set<std::pair<double,std::vector<lanelet_ptr_t> > > LaneletMap::possible_routes_set(const lanelet_ptr_t& start, double horizon_metric, bool filter_out_lane_changes) const
{
  //! @note: non performant search! @todo: replace this with better solution
  std::set<std::pair<double,std::vector<lanelet_ptr_t> > > result;
  std::vector<lanelet_ptr_t> temp_llet_vector;
  std::set<lanelet_ptr_t> following_set;


  temp_llet_vector.clear();
  temp_llet_vector.push_back(start);
  result.insert(std::make_pair(start->length(LLet::CENTER), temp_llet_vector));

  std::set<std::pair<double,std::vector<lanelet_ptr_t> > >::iterator it = result.begin();
  while (it!=result.end())
  {
    std::pair<double,std::vector<lanelet_ptr_t> > route = *it;
    if (route.first < horizon_metric)
    {
      following_set = LaneletMap::following_set(route.second.back(), filter_out_lane_changes);
      bool following_lane_exists = false;
      for (std::set<lanelet_ptr_t>::iterator it2=following_set.begin(); it2!=following_set.end(); ++it2)
      {
        following_lane_exists = true;
        route = *it;
        lanelet_ptr_t temp_llet = *it2;
        route.second.push_back(*it2);
        route.first += temp_llet->length(LLet::CENTER);
        result.insert(route);
      }
      if (following_lane_exists)
      {
        result.erase(it);
        it=result.begin();
      }
      else
      {
        ++it;
      }
    }
    else
    {
      ++it;
    }
  }
  return result;
}


std::set<std::pair<double,std::vector<lanelet_ptr_t> > > LaneletMap::possible_routes_set(const lanelet_ptr_t& start, int horizon_count, bool filter_out_lane_changes) const
{
  //! @note: non performant search! @todo: replace this with better solution
  std::set<std::pair<double,std::vector<lanelet_ptr_t> > > result;
  std::vector<lanelet_ptr_t> temp_llet_vector;
  std::set<lanelet_ptr_t> following_set;

  temp_llet_vector.clear();
  temp_llet_vector.push_back(start);
  result.insert(std::make_pair(start->length(LLet::CENTER), temp_llet_vector));

  std::set<std::pair<double,std::vector<lanelet_ptr_t> > >::iterator it = result.begin();
  while (it!=result.end())
  {
    std::pair<double,std::vector<lanelet_ptr_t> > route = *it;
    if (static_cast<int>(route.second.size()) < horizon_count)
    {
      following_set = LaneletMap::following_set(route.second.back(), filter_out_lane_changes);
      bool following_lane_exists = false;
      for (std::set<lanelet_ptr_t>::iterator it2=following_set.begin(); it2!=following_set.end(); ++it2)
      {
        following_lane_exists = true;
        route = *it;
        lanelet_ptr_t temp_llet = *it2;
        route.second.push_back(*it2);
        route.first += temp_llet->length(LLet::CENTER);
        result.insert(route);
      }
      if (following_lane_exists)
      {
        result.erase(it);
        it=result.begin();
      }
      else
      {
        ++it;
      }
    }
    else
    {
      ++it;
    }
  }

  return result;
}

std::set<std::pair<double,std::vector<lanelet_ptr_t> > > LaneletMap::possible_routes_set(const lanelet_ptr_t& start, int horizon_previous, int horizon_following, bool filter_out_lane_changes) const
{
  std::set<std::pair<double,std::vector<lanelet_ptr_t> > > result;

  //! @todo: implement version for alternate values of horizon_previous and horizon_following
  if (horizon_previous != 1 || horizon_following != 1)
  {
    std::cout << "Function not yet implemented for given set of parameters (other parameters than horizon_previous=1 and horizon_following=1 are currently not allowed)" << std::endl;
    return result;
  }

  std::set<lanelet_ptr_t> previous_set = LaneletMap::previous_set(start,filter_out_lane_changes);
  std::set<lanelet_ptr_t> following_set = LaneletMap::following_set(start,filter_out_lane_changes);

  if (previous_set.size() == 0)
  {
    lanelet_ptr_t previous_lanelet = lanelet_ptr_t();
    if (following_set.size() == 0)
    {
      lanelet_ptr_t following_lanelet = lanelet_ptr_t();
      std::vector<lanelet_ptr_t> temp_llet_vector;
      temp_llet_vector.push_back(previous_lanelet);
      temp_llet_vector.push_back(start);
      temp_llet_vector.push_back(following_lanelet);
      double length = start->length(LLet::CENTER);
      result.insert(std::make_pair(length,temp_llet_vector));
    }
    else
    {
      for (std::set<lanelet_ptr_t>::iterator it2 = following_set.begin(); it2 != following_set.end(); ++it2)
      {
        std::vector<lanelet_ptr_t> temp_llet_vector;
        temp_llet_vector.push_back(previous_lanelet);
        temp_llet_vector.push_back(start);
        temp_llet_vector.push_back(*it2);
        double length = start->length(LLet::CENTER) + (*it2)->length(LLet::CENTER);
        result.insert(std::make_pair(length,temp_llet_vector));
      }
    }
  }
  else
  {
    for (std::set<lanelet_ptr_t>::iterator it = previous_set.begin(); it != previous_set.end(); ++it)
    {
      if (following_set.size() == 0)
      {
        lanelet_ptr_t following_lanelet = lanelet_ptr_t();
        std::vector<lanelet_ptr_t> temp_llet_vector;
        temp_llet_vector.push_back(*it);
        temp_llet_vector.push_back(start);
        temp_llet_vector.push_back(following_lanelet);
        double length = (*it)->length(LLet::CENTER) + start->length(LLet::CENTER);
        result.insert(std::make_pair(length,temp_llet_vector));
      }
      else
      {
        for (std::set<lanelet_ptr_t>::iterator it2 = following_set.begin(); it2 != following_set.end(); ++it2)
        {
          std::vector<lanelet_ptr_t> temp_llet_vector;
          temp_llet_vector.push_back(*it);
          temp_llet_vector.push_back(start);
          temp_llet_vector.push_back(*it2);
          double length = (*it)->length(LLet::CENTER) + start->length(LLet::CENTER) + (*it2)->length(LLet::CENTER);
          result.insert(std::make_pair(length,temp_llet_vector));
        }
      }
    }
  }
  return result;
}

std::set<lanelet_ptr_t> LaneletMap::beside_set(const lanelet_ptr_t &start) const
{
  std::set<lanelet_ptr_t> result;
  Graph::out_edge_iterator iter, end;
  int64_t current_id = vertex_id_by_lanelet(start);
  for (boost::tie(iter, end) = out_edges(current_id, _graph); iter != end; ++iter)
  {
    lanelet_ptr_t lanelet = _graph[boost::target(*iter, _graph)].lanelet;
    result.insert(lanelet);
  }
  for (std::set<lanelet_ptr_t>::iterator it=result.begin(); it!=result.end(); /* */)
  {
    if (start->fits_before(*it))
    {
      result.erase(it++); //implicitly save a copy of it
    }
    else
    {
      ++it;
    }
  }

  return result;
}

corridor_ptr_t LaneletMap::corridor(const lanelet_ptr_t &start, const lanelet_ptr_t &end) const
{
  return corridor_ptr_t(new Corridor(start, end, this));
}

const lanelet_ptr_t &LaneletMap::lanelet_by_id(int64_t id) const
{

    // Replaced by the code below
//    std::vector< lanelet_ptr_t >::const_iterator pos = std::find_if(
//                                                         _lanelets.begin(),
//                                                         _lanelets.end(),
//                                                         [&id](const lanelet_ptr_t& ll) {return ll->id() == id;});
    std::vector< lanelet_ptr_t >::const_iterator pos = _lanelets.begin();
    for ( ; pos < _lanelets.end(); ++pos)
    {
      if (pos->get()->id() == id)
      {
        break;
      }
    }


    if( pos != _lanelets.end() )
        return *pos;
    else
    {
        boost::format fmt("trying to retieve lanelet with unknown id: %i");
        throw std::runtime_error( (fmt % id).str() );
    }
}

const Graph &LaneletMap::graph() const
{
  return _graph;
}

const std::vector<lanelet_ptr_t>& LaneletMap::lanelets() const
{
  return _lanelets;
}

const std::vector<parking_space_ptr_t>& LaneletMap::parking_spaces() const
{
  return _parking_spaces;
}

bool LaneletMap::add_parking_space(parking_space_ptr_t parking_space){
  _parking_spaces.push_back(parking_space);
  return true;
}

const std::vector<event_region_ptr_t>& LaneletMap::event_regions() const
{
  return _event_regions;
}

const std::vector< traffic_light_ptr_t >& LaneletMap::traffic_lights() const
{
  return _traffic_lights;
}

int64_t LaneletMap::get_minimum_node_id(){
  return --_minimum_node_id;
}

void LaneletMap::bounding_box(const point_with_id_t& gnss_reference_point, point_xy_t& lower_left, point_xy_t& upper_right) const
{
  assert((lanelets().empty() == false) && "There is no bounding box for an empty map");
  std::vector<point_xy_t> corner_points;

  // aggregate bounding boxes' corner points of all lanelets
  point_xy_t ll, ur;
  point_with_id_t ll_gnss, ur_gnss;
  for (std::size_t i=0; i<lanelets().size(); ++i)
  {
    lanelets()[i]->polygon()->boundingBox(ll, ur);

    // convert to gnss using lanelet's internal reference point
    ll_gnss = from_vec(lanelets()[i]->polygon()->gnssReferencePoint(), ll);
    ur_gnss = from_vec(lanelets()[i]->polygon()->gnssReferencePoint(), ur);

    // convert to metric using external reference point
    corner_points.push_back(vec(gnss_reference_point, ll_gnss));
    corner_points.push_back(vec(gnss_reference_point, ur_gnss));
  }

  // determine bounding box of the map
  assert((corner_points.empty() == false) && "Something went wrong when extracting bounding box corner points out of lanelets!");
  double min_x = boost::get<X>(corner_points.front());
  double max_x = min_x;
  double min_y = boost::get<Y>(corner_points.front());
  double max_y = min_y;

  for (std::vector<point_xy_t>::const_iterator point = corner_points.begin()+1; point < corner_points.end(); ++point)
  {
    min_x = std::min(min_x, boost::get<X>(*point));
    max_x = std::max(max_x, boost::get<X>(*point));

    min_y = std::min(min_y, boost::get<Y>(*point));
    max_y = std::max(max_y, boost::get<Y>(*point));
  }

  boost::get<X>(lower_left) = min_x;
  boost::get<Y>(lower_left) = min_y;

  boost::get<X>(upper_right) = max_x;
  boost::get<Y>(upper_right) = max_y;
}

void LaneletMap::init()
{
    for( std::size_t i = 0; i < _lanelets.size(); ++i )
    {
        Graph::vertex_descriptor vtx = boost::add_vertex(_graph);
        _graph[vtx].lanelet = _lanelets[i];
        // graph vertex descriptors and _lanelet indices should be in sync now.
        assert( vtx == i );
        assert( vertex_id_by_lanelet(_lanelets[i]) == int(i) );
        _lanelet_tree.insert(_lanelets[i]);
    }

    BOOST_FOREACH( const lanelet_ptr_t& src, _lanelets )
    {
        int64_t index_src = this->vertex_id_by_lanelet(src);
        double len = src->length();
        std::vector<lanelet_ptr_t> lls_around = this->query( src->bb() );
        BOOST_FOREACH( const lanelet_ptr_t& other, lls_around )
        {
            bool const is_predecessor = src->fits_before(other);
            if(is_predecessor || src->fits_beside(other, LEFT) || src->fits_beside(other, RIGHT))
            {
                int64_t index_dest = this->vertex_id_by_lanelet( other );
                bool inserted;
                Graph::edge_descriptor new_edge;
                boost::tie(new_edge, inserted) = boost::add_edge(index_src, index_dest, this->_graph);
                assert(inserted);
                EdgeInfo info;
                // Routing cost is fixed for lateral connected ones (parallel lanelets) and lanelet length otherwise
                info.routing_cost = is_predecessor ? len : 2.0;
                _graph[new_edge] = info;
            }
        }
    }

}

int64_t LaneletMap::vertex_id_by_lanelet(const lanelet_ptr_t &lanelet) const
{
    std::vector< lanelet_ptr_t >::const_iterator pos = std::find(_lanelets.begin(), _lanelets.end(), lanelet);
    if( pos == _lanelets.end() )
    {
	std::stringstream ss;
        for (std::vector<lanelet_ptr_t>::const_iterator pos=_lanelets.begin(); pos != _lanelets.end(); ++pos)
        {
          if ((*pos)->id() == lanelet->id())
          {
            ss << "Found lanelet id=" << lanelet->id() << " by id comparison. lanelet_ptr=" << lanelet << " map ptr " << *pos << std::endl;
  	    throw std::runtime_error(ss.str());
          }
        }
        ss << "Lanelet with id=" << lanelet->id() << " not found."; 
        throw std::runtime_error(ss.str());
    }
    else
        return std::distance(_lanelets.begin(), pos);
}


std::set<LLet::lanelet_ptr_t> LLet::LaneletMap::lanelet_by_covered_point(const point_with_id_t& query) const
{
  std::set<LLet::lanelet_ptr_t> result;
  for (std::size_t i=0; i<_lanelets.size(); ++i)
  {
    if (_lanelets[i]->covers_point(query))
    {
      result.insert(_lanelets[i]);
    }
  }
  return result;
}

point_with_id_t LaneletMap::map_matching(const point_with_id_t &source, lanelet_ptr_t &lanelet, double max_distance, double *heading, bool check_heading, double vehicle_heading, double threshold) const
{
  double heading_value = 0; // temp variable whose pointer might be needed later in project()
  bool reset_heading = false; // needed to check whether heading has to be reset to a nullptr

  if (check_heading && !heading)
  {
    // create a temp pointer to heading to be able to compare with vehicle_heading, as paramter heading doesn't exist yet
    heading = &heading_value;
    reset_heading = true;
  }

  if (lanelet && lanelet->covers_point(source))
  {
    point_with_id_t result = lanelet->project(source, heading);
    if (!check_heading || std::abs(*heading - vehicle_heading) < threshold)
    {
      heading = (reset_heading) ? NULL : heading;
      return result;
    }
  }

  double north, west, south, east;
  double const lat = boost::get<LLet::LAT>(source);
  double const lon = boost::get<LLet::LON>(source);
  convert_coordinates::latlon_add_meters(lat, lon, max_distance, max_distance, north, east );
  convert_coordinates::latlon_add_meters(lat, lon, -max_distance, -max_distance, south, west);
  BoundingBox const box( boost::make_tuple(south, west, north, east));
  std::vector<lanelet_ptr_t> const lanelets = query(box);

  BOOST_FOREACH(const lanelet_ptr_t &llet, lanelets)
  {
    if (llet->covers_point(source))
    {
      point_with_id_t result = llet->project(source, heading);
      if (!check_heading || std::abs(*heading - vehicle_heading) < threshold)
      {
        lanelet = llet;
        heading = (reset_heading) ? NULL : heading;
        return result;
      }
    }
  }

  double min_distance = max_distance;
  lanelet_ptr_t result;
  BOOST_FOREACH(const lanelet_ptr_t &llet, lanelets)
  {
    double const distance = llet->distance_to(source);
    llet->project(source, heading);
    if (distance < min_distance && (!check_heading || std::abs(*heading - vehicle_heading) < threshold))
    {
      min_distance = distance;
      result = llet;
    }
  }

  if (result)
  {
    lanelet = result;
    heading = (reset_heading) ? NULL : heading;
    return result->project(source, heading);
  }

  lanelet = lanelet_ptr_t();
  heading = (reset_heading) ? NULL : heading;
  return source;
}

double LaneletMap::score_lanelet(double metric_distance, double orientation_distance) const
{
  double metric_mean = 0.;
  double metric_stddev = 0.5;
  double orientation_mean = 1.;
  double orientation_stddev = 0.3;

  //double metric_score = exp(-pow(metric_distance-metric_mean,2)/(2*pow(metric_stddev,2)))/(metric_stddev*sqrt(2*M_PI));
  //double orientation_score = exp(-pow(orientation_distance-orientation_mean,2)/(2*pow(orientation_stddev,2)))/(orientation_stddev*sqrt(2*M_PI));
  double metric_score = exp(-pow(metric_distance-metric_mean,2)/(2*pow(metric_stddev,2)));
  double orientation_score = exp(-pow(orientation_distance-orientation_mean,2)/(2*pow(orientation_stddev,2)));
  return metric_score * orientation_score;
}

void LaneletMap::map_matching_probabilistic(const point_with_id_t &source, std::vector<std::pair<double,lanelet_ptr_t> > &lanelet_list, double max_distance, boost::tuple< double, double > vehicle_velocity, double threshold) const
{
  bool use_orientation = false;
  if (boost::get<X>(vehicle_velocity) != 0 || boost::get<Y>(vehicle_velocity) != 0)
  {
    normalize(vehicle_velocity);
    use_orientation = true;
  }

  lanelet_list.clear();
  double north, west, south, east;
  double const lat = boost::get<LLet::LAT>(source);
  double const lon = boost::get<LLet::LON>(source);
  convert_coordinates::latlon_add_meters(lat, lon, max_distance, max_distance, north, east );
  convert_coordinates::latlon_add_meters(lat, lon, -max_distance, -max_distance, south, west);
  BoundingBox const box( boost::make_tuple(south, west, north, east));
  std::vector<lanelet_ptr_t> const lanelets = query(box);

  BOOST_FOREACH(const lanelet_ptr_t &llet, lanelets)
  {
    std::size_t index;
    std::size_t previous_index;
    std::size_t subsequent_index;
    double angle;
    double metric_distance = llet->distance_to(source);

    if (llet->covers_point(source))
    {
      metric_distance = 0;
    }

    llet->project(source, &angle, &index, &previous_index, &subsequent_index);
    boost::tuple< double, double > lanelet_vector = vec(llet->node_at(CENTER, previous_index), llet->node_at(CENTER, subsequent_index));
    normalize(lanelet_vector);
    double orientation_distance;
    if (use_orientation)
    {
      orientation_distance = scalar_product(lanelet_vector,vehicle_velocity);
    }
    else
    {
      orientation_distance = 1; // as we don't know the orientation, just assume it fits perfectly
    }
    double score = score_lanelet(metric_distance, orientation_distance);
    if (score >= threshold)
    {
      lanelet_list.push_back(std::make_pair(score,llet));
    }
  }

  double cumulative_score = 0;
  for (std::vector<std::pair<double,lanelet_ptr_t> >::iterator it = lanelet_list.begin(); it != lanelet_list.end(); ++it)
  {
    cumulative_score += it->first;
  }
  for (std::vector<std::pair<double,lanelet_ptr_t> >::iterator it = lanelet_list.begin(); it != lanelet_list.end(); ++it)
  {
    it->first = it->first / cumulative_score;
  }
}

void LaneletMap::map_matching_probabilistic(const LLet::point_with_id_t &source, LaneletByProbabilityMap &lanelet_map, double max_distance, boost::tuple<double, double> vehicle_velocity, double threshold) const
{
  std::vector<std::pair<double,lanelet_ptr_t> > lanelet_list;
  map_matching_probabilistic(source, lanelet_list, max_distance, vehicle_velocity, threshold);

  typedef std::pair<double,lanelet_ptr_t> DoubleLaneletPair;
  BOOST_FOREACH(DoubleLaneletPair pair, lanelet_list)
  {
    lanelet_map.insert(pair);
  }

  return;
}

TiXmlDocument LaneletMap::toXml()
{
  TiXmlDeclaration* declaration = new TiXmlDeclaration("1.0", "UTF-8", "");
  TiXmlDocument doc;
  doc.LinkEndChild(declaration);

  TiXmlElement osm("osm");
  osm.SetAttribute("version", "0.6");
  osm.SetAttribute("generator", "liblanelet");
  TiXmlNode* osm_node(doc.InsertEndChild(osm));

  //collect all nodes, regulatory elements and ways
  ID_Visitor visitor;
  int64_t stripCounter = -1;
  int64_t regulatory_element_counter = -1;
  std::map<int64_t, LLet::point_with_id_t> points;
  std::map<LLet::regulatory_element_ptr_t, int64_t> regulatory_element_ids;
  std::map<int, LLet::regulatory_element_ptr_t> regulatory_elements;
  BOOST_FOREACH(LLet::lanelet_ptr_t lanelet, _lanelets)
  {
    regulatory_element_counter = std::min(regulatory_element_counter,
                                          lanelet->id() - 1);
  }
  BOOST_FOREACH(boost::shared_ptr<LLet::Lanelet> lanelet, _lanelets)
  {
    const std::vector<LLet::point_with_id_t> left_points = lanelet->nodes(LLet::LEFT);
    BOOST_FOREACH(LLet::point_with_id_t point, left_points)
    {
      points.insert(std::make_pair(boost::get<LLet::ID>(point), point));
    }
    const std::vector<LLet::point_with_id_t> right_points = lanelet->nodes(LLet::RIGHT);
    BOOST_FOREACH(LLet::point_with_id_t point, right_points)
    {
      points.insert(std::make_pair(boost::get<LLet::ID>(point), point));
    }

    LLet::strip_ptr_t left_strip = boost::get<LLet::LEFT>(lanelet->bounds());
    visitor.insert(left_strip, stripCounter--);
    LLet::strip_ptr_t right_strip = boost::get<LLet::RIGHT>(lanelet->bounds());
    visitor.insert(right_strip, stripCounter--);

    BOOST_FOREACH(LLet::regulatory_element_ptr_t regElem, lanelet->regulatory_elements())
    {
      int regElemId = regulatory_element_counter--;
      regulatory_elements.insert(std::make_pair(regElemId, regElem));
      regulatory_element_ids.insert(std::make_pair(regElem, regElemId));
    }
  }
  //Check if there is a way in regulatory elements that is not covered by a lanelet and add it
  typedef std::pair<int, LLet::regulatory_element_ptr_t> RegulatoryElementValue;
  typedef std::pair<std::string, LLet::member_variant_t> MemberValue;
  BOOST_FOREACH(RegulatoryElementValue item, regulatory_elements) {
    BOOST_FOREACH(MemberValue pair, item.second->members()) {

      std::string ref = boost::apply_visitor(visitor, pair.second);
      if(ref.length() == 0 && boost::apply_visitor(type_visitor(), pair.second) == "way") {

        LLet::strip_ptr_t missingWay = boost::get<LLet::strip_ptr_t>(pair.second);
        visitor.insert(boost::get<LLet::strip_ptr_t>(pair.second), --stripCounter);
        ref =  boost::apply_visitor(visitor, pair.second);
        BOOST_FOREACH(point_with_id_t& point, missingWay->pts()) {
          points.insert(std::make_pair(boost::get<LLet::ID>(point), point));

        }
      }
    }
  }

  //store all nodes in XML
  //Check it there are nodes in parking spaces that are not covered yet
  BOOST_FOREACH(parking_space_ptr_t parking_space, _parking_spaces)
  {
    BOOST_FOREACH(point_with_id_t point, parking_space->getVertices())
    {
        points.insert(std::make_pair(boost::get<LLet::ID>(point), point));
    }
  }

  typedef std::map<int64_t, LLet::point_with_id_t>::value_type PointPair;
  BOOST_FOREACH(PointPair pair, points) {
      TiXmlElement node("node");
      node.SetAttribute("id", boost::lexical_cast<std::string>(boost::get<LLet::ID>(pair.second)));
      node.SetAttribute("visible", "true");
      node.SetAttribute("lat", boost::lexical_cast<std::string>(boost::get<LLet::LAT>(pair.second)));
      node.SetAttribute("lon", boost::lexical_cast<std::string>(boost::get<LLet::LON>(pair.second)));
      node.SetAttribute("version", "1");
      osm_node->InsertEndChild(node);
    }


  //store all strips in XML
  typedef std::map<LLet::strip_ptr_t, int64_t>::value_type StripPair;
  BOOST_FOREACH(StripPair pair, visitor.ways()) {
      TiXmlElement way("way");
      way.SetAttribute("id", visitor.ways().at(pair.first));
      way.SetAttribute("visible", "true");
      way.SetAttribute("version", "1");
      TiXmlNode* way_node(osm_node->InsertEndChild(way));

      BOOST_FOREACH(LLet::point_with_id_t point, pair.first->pts()) {
          TiXmlElement nd("nd");
          nd.SetAttribute("ref", boost::get<LLet::ID>(point));
          way_node->InsertEndChild(nd);
        }
    }

  //store the lanelets
  BOOST_FOREACH(boost::shared_ptr<LLet::Lanelet> lanelet, _lanelets) {
      LLet::strip_ptr_t left_strip = boost::get<LLet::LEFT>(lanelet->bounds());
      LLet::strip_ptr_t right_strip = boost::get<LLet::RIGHT>(lanelet->bounds());

      TiXmlElement relation("relation");
      relation.SetAttribute("id", lanelet->id());
      relation.SetAttribute("visible", "true");
      relation.SetAttribute("version","1");
      TiXmlNode* relation_node(osm_node->InsertEndChild(relation));

      TiXmlElement memberLeft("member");
      memberLeft.SetAttribute("type", "way");
      memberLeft.SetAttribute("ref", visitor.ways().at(left_strip));
      memberLeft.SetAttribute("role", "left");

      relation_node->InsertEndChild(memberLeft);

      TiXmlElement memberRight("member");
      memberRight.SetAttribute("type", "way");
      memberRight.SetAttribute("ref", visitor.ways().at(right_strip));
      memberRight.SetAttribute("role", "right");
      relation_node->InsertEndChild(memberRight);

      TiXmlElement tag("tag");
      tag.SetAttribute("k", "type");
      tag.SetAttribute("v", "lanelet");
      relation_node->InsertEndChild(tag);

      //store lanelet-tags
      const AttributeMap& attributes = lanelet->attributes();
      BOOST_FOREACH( const AttributeMap::value_type& a, attributes )
      {
          //ignore lanelet-tag itself because it is handled above
          if (!(a.first == "type" && a.second.as_string() == "lanelet")) {
              TiXmlElement tag("tag");
              tag.SetAttribute("k", a.first);
              tag.SetAttribute("v", a.second.as_string());
              relation_node->InsertEndChild(tag);
          }

      }


      //store the regulatory elements
      BOOST_FOREACH(LLet::regulatory_element_ptr_t regElem, lanelet->regulatory_elements()) {
          TiXmlElement regElemRelation("member");
          regElemRelation.SetAttribute("type", "relation");
          regElemRelation.SetAttribute("ref", regulatory_element_ids[regElem]);
          regElemRelation.SetAttribute("role", "regulatory_element");
          relation_node->InsertEndChild(regElemRelation);
        }
    }

  typedef std::pair<std::string, LLet::AttributeValue> AttributePair;
  BOOST_FOREACH(RegulatoryElementValue item, regulatory_elements) {
      TiXmlElement regElemRelation("relation");
      regElemRelation.SetAttribute("id", item.first);
      regElemRelation.SetAttribute("visible", "true");
      TiXmlNode* regElemNode(osm_node->InsertEndChild(regElemRelation));
      TiXmlElement typeTag("tag");
      typeTag.SetAttribute("k", "type");
      typeTag.SetAttribute("v", "regulatory_element");
      regElemNode->InsertEndChild(typeTag);
      BOOST_FOREACH(AttributePair pair, item.second->attributes()) {
          TiXmlElement tag("tag");
          tag.SetAttribute("k", pair.first);
          tag.SetAttribute("v", pair.second.as_string());
          regElemNode->InsertEndChild(tag);
        }
      BOOST_FOREACH(MemberValue pair, item.second->members()) {
          TiXmlElement member("member");
          member.SetAttribute("role", pair.first);
          member.SetAttribute("ref", boost::apply_visitor(visitor, pair.second));
          member.SetAttribute("type", boost::apply_visitor(type_visitor(), pair.second));
          regElemNode->InsertEndChild(member);
        }
    }

  //store parking_spaces
  BOOST_FOREACH(parking_space_ptr_t parking_space, parking_spaces())
  {
    //1.create WAY node
    TiXmlElement parkingSpaceElem("way");
    parkingSpaceElem.SetAttribute("id",parking_space->getId());
    parkingSpaceElem.SetAttribute("visible","true");
    parkingSpaceElem.SetAttribute("version","1");
    TiXmlNode* parkingSpaceWayNode(osm_node->InsertEndChild(parkingSpaceElem));

    //1.1 add Amenity
    TiXmlElement amenityElem("tag");
    amenityElem.SetAttribute("k", "amenity");
    amenityElem.SetAttribute("v", "parking_space");
    parkingSpaceWayNode->InsertEndChild(amenityElem);

    //1.2 add parking direction node
    TiXmlElement directionElem("tag");
    directionElem.SetAttribute("k", "parking_direction");
    if (parking_space->getParkingDirection() == ParkingSpace::BOTH)
    {
      directionElem.SetAttribute("v", "both");
    }
    else if (parking_space->getParkingDirection() == ParkingSpace::FORWARD)
    {
      directionElem.SetAttribute("v", "forward");;
    }
    else if (parking_space->getParkingDirection() == ParkingSpace::BACKWARD)
    {
      directionElem.SetAttribute("v", "forward");;
    }
    parkingSpaceWayNode->InsertEndChild(directionElem);

    //2 insert nodes
    BOOST_FOREACH(point_with_id_t point, parking_space->getVertices())
    {
      TiXmlElement wayPointElem("nd");
      wayPointElem.SetAttribute("ref",boost::lexical_cast<std::string>(boost::get<LLet::ID>(point)));
      parkingSpaceWayNode->InsertEndChild(wayPointElem);
    }

    //2.1 Create Relation to lanlet node
    if(parking_space->getRelatedLanelets().size() > 0)
    {
      TiXmlElement relatedLaneletElem("relation");
      relatedLaneletElem.SetAttribute("id",get_minimum_node_id());
      relatedLaneletElem.SetAttribute("visible","true");
      relatedLaneletElem.SetAttribute("version","1");
      TiXmlNode* relatedLaneletNode(osm_node->InsertEndChild(relatedLaneletElem));

      //2.2 Add TypeTag
      TiXmlElement typeTagElement("tag");
      typeTagElement.SetAttribute("k", "type");
      typeTagElement.SetAttribute("v", "accessible");
      relatedLaneletNode->InsertEndChild(typeTagElement);


      BOOST_FOREACH(int64_t lanelet_id, parking_space->getRelatedLanelets())
      {
        TiXmlElement memberLanletElement("member");
        memberLanletElement.SetAttribute("type","relation");
        memberLanletElement.SetAttribute("ref",lanelet_id);
        memberLanletElement.SetAttribute("role","lanelet");
        relatedLaneletNode->InsertEndChild(memberLanletElement);
      }

      //2.4 Add parking space as memeber
      TiXmlElement memberParkingSpaceElement("member");
      memberParkingSpaceElement.SetAttribute("type","way");
      memberParkingSpaceElement.SetAttribute("ref",parking_space->getId());
      memberParkingSpaceElement.SetAttribute("role","parking_space");
      relatedLaneletNode->InsertEndChild(memberParkingSpaceElement);
    }

  }


  return doc;
}

