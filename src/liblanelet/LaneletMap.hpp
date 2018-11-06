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

#pragma once

#include "Lanelet.hpp"
#include "ParkingSpace.hpp"
#include "EventRegion.hpp"
#include "TrafficLight.hpp"
#include "LLTree.hpp"
#include "LaneletGraph.hpp"
#include "Corridor.hpp"
#include "ImportExport.h"
#include <tinyxml.h>

namespace LLet
{

struct NoPath
{
    lanelet_ptr_t start;
    lanelet_ptr_t dest;
};

/*!
 * \brief
 * /EventRegion/ParkingSpace.
 * To assign IDs to LineStrips (which don't store an id themselves), this class
 * is extended by a map that assigns ids to LineStrips.
 */
class ID_Visitor : public boost::static_visitor<std::string> {
public:
  std::string operator()(LLet::lanelet_ptr_t lanelet) const {
    return boost::lexical_cast<std::string>(lanelet->id());
  }

  std::string operator()(LLet::strip_ptr_t strip) const {
    if (m_linestrips.count(strip) > 0) {
      return boost::lexical_cast<std::string>(m_linestrips.at(strip));
    } else {
      return "";
    }
  }

  std::string operator()(LLet::point_with_id_t point) const {
    return boost::lexical_cast<std::string>(boost::get<LLet::ID>(point));
  }

  std::string operator()(LLet::parking_space_ptr_t  parking_space) const {
    return boost::lexical_cast<std::string>(parking_space->getId());
  }

  std::string operator()(LLet::event_region_ptr_t  event_region) const {
    return boost::lexical_cast<std::string>(event_region->getId());
  }

  /*!
   * \brief insert Inserts a new LineStrip with the given id.
   * \param strip The LineStrip.
   * \param id The id of the LineStrip.
   */
  void insert(LLet::strip_ptr_t strip, int64_t id) { m_linestrips.insert(std::make_pair(strip, id)); }

  /*!
   * \brief ways Returns the map of all stored LineStrips with their ids.
   * \return A map of all stored LineStrips with their ids.
   */
  std::map<LLet::strip_ptr_t, int64_t> ways() { return m_linestrips; }

private:
  std::map<LLet::strip_ptr_t, int64_t> m_linestrips;
};


/*!
 * \brief The type_visitor class returns the OSM type of the lanelet/strip/point.
 */
class type_visitor : public boost::static_visitor<std::string> {
public:
  std::string operator()(LLet::lanelet_ptr_t lanelet) const {
    return "relation";
  }

  std::string operator()(LLet::strip_ptr_t strip) const {
    return "way";
  }

  std::string operator()(LLet::point_with_id_t point) const {
    return "node";
  }

  std::string operator()(LLet::event_region_ptr_t event_region) const {
    return "way";
  }

  std::string operator()(LLet::ParkingSpace parking_space) const {
    return "way";
  }

  std::string operator()(LLet::EventRegion event_region) const {
    return "way";
  }
};

class LANELET_IMPORT_EXPORT LaneletMap
{
public:

    struct ReachableSetInfo
    {
      int64_t index;
      double total_distance;
      explicit ReachableSetInfo(int64_t index_=-1, double distance=0.0) : index(index_), total_distance(distance) {}
      bool operator<(const ReachableSetInfo &other) const { return total_distance > other.total_distance; }
    };

    typedef std::multimap<double, lanelet_ptr_t> LaneletByProbabilityMap;
    typedef std::vector< lanelet_ptr_t > lanelet_ptr_vector;

    LaneletMap( const std::vector< lanelet_ptr_t > &lanelets );
    LaneletMap(const std::string &filename , const bool ignore_consistency_failures=false);

    std::vector< lanelet_ptr_t > query( const BoundingBox& box ) const;

    /**
     * @brief shortest_paths Calculates the shortest path between \a start lanelet and a single \a destination lanelet.
     * @param start The lanelet to start
     * @param dest The destination lanelet
     * @return A vector of lanelets (path) from \a start to \a dest
     */
    std::vector< lanelet_ptr_t > shortest_path( const lanelet_ptr_t& start, const lanelet_ptr_t& dest ) const;

    /**
     * @brief shortest_path Determines the shortest path of a given set of \a paths. A path
     * consits of mutiple lanelets. The length of a single path consits of the lenght of the center
     * lines of the lanelets.
     * @param paths A set of paths
     * @return The shortest path of a given set. If two paths of a set have the same length then only the
     * first one in the set will returned.
     */
    lanelet_ptr_vector shortest_path(const std::vector<lanelet_ptr_vector> &paths ) const;

    /**
     * @brief shortest_paths Calculates the shortest path between \a start lanelet and mutiple \a destinations lanelets.
     * @param start The lanelet to start
     * @param destinations A set with mutiple destinations
     * @return A set of lanelets (path) for each destination.
     */
    lanelet_ptr_vector shortest_paths(const lanelet_ptr_vector &starts,
                                                   const lanelet_ptr_vector &destinations ) const;

 
    /**
     * @brief shortest_paths Calculates the shortest path between \a start lanelet and mutiple \a destinations lanelets.
     * @param start The lanelet to start
     * @param destinations A set with mutiple destinations
     * @return A set of lanelets (path) for each destination.
     */
    std::vector<lanelet_ptr_vector> shortest_paths(const lanelet_ptr_t& start,
                                                   const lanelet_ptr_vector &destinations ) const;

    /**
     * @brief shortest_path Find the shortest path between \a start lanelet and \a destination parking_spaces.
     * A parking space can be related to more than one lanelet. Every related lanelet is viewed as a destination
     * during rounting.
     * @param start The lanelet to start
     * @param destination The destination parking space
     * @return A set of lanelets (path) from start to destination. Or an empty vector if there is no path.
     */
    lanelet_ptr_vector shortest_path(const lanelet_ptr_t& start, const parking_space_ptr_t& destination ) const;

    /**
     * @brief reachable_set Find the lanelets that can be reached from the given lanelet without travelling more than max_distance (distance along lanelets)
     * @param start Source lanelet
     * @param max_distance Maximum allowed distance between source and target lanelets
     * @return A set of target lanelets that are reachable from source and have a distance less than maximum_distance
     * @note Not yet tested with large distances (huge maps and large output sets)
     */
    std::set<lanelet_ptr_t> reachable_set(const lanelet_ptr_t& start, double max_distance) const;


    /*! Determine the set of lanelets that are following the \a start lanelet in the graph of lanelets.
     *  E.g. all lateral and longitudinal lanelet neighbors are found.
     *  \param start The lanelet to start
     *  \param filter_out_lane_changes When set to \c true only longitudinal connections
     *         are found (lane changes are filtered out). \note lane != lanelet
     *  \return A set of lanelets that are directly reachable from \a start
     */
    std::set<lanelet_ptr_t> following_set(const lanelet_ptr_t& start, bool filter_out_lane_changes = false) const;

    /*! Determine the set of lanelets that are previous of the \a start lanelet in the graph of lanelets.
     *  E.g. all lateral and longitudinal lanelet neighbors are found.
     *  \param start The lanelet to start
     *  \param filter_out_lane_changes When set to \c true only longitudinal connections
     *         are found (lane changes are filtered out). \note lane != lanelet
     *  \return A set of lanelets from which \a start is directly reachable
     */
    std::set<lanelet_ptr_t> previous_set(const lanelet_ptr_t& start, bool filter_out_lane_changes = false) const;


    /*! Determine the set of routes(one route is a vector of consecutive lanelets) which are possible to follow from the \a start lanelet in the graph of lanelets.
     *  \param start The lanelet to start
     *  \param horizon_metric The minimum length of the route
     *  \param filter_out_lane_changes Used for following_set() function call
     *  \return A set of routes that are directly reachable from \a start (actually the return value consists of a std::pair containing the route length and the route itself)
     *  @note This may not be the most performant solution! Think about solving this with the Graph directly. Don't use this with too long distances
     * (depending on the length of the lanelets) if you need fast results
     */
    std::set<std::pair<double,std::vector<lanelet_ptr_t> > > possible_routes_set(const lanelet_ptr_t& start, double horizon_metric, bool filter_out_lane_changes) const;

    /*! Determine the set of routes(one route is a vector of consecutive lanelets) which are possible to follow from the \a start lanelet in the graph of lanelets.
     *  \param start The lanelet to start
     *  \param horizon_metric The number of lanelets forming the resulting route
     *  \param filter_out_lane_changes Used for following_set() function call
     *  \return A set of routes that are directly reachable from \a start (actually the return value consists of a std::pair containing the route length and the route itself)
     *  @note This may not be the most performant solution! Think about solving this with the Graph directly. Don't use this with too long distances
     * (depending on the length of the lanelets) if you need fast results
     */
    std::set<std::pair<double,std::vector<lanelet_ptr_t> > > possible_routes_set(const lanelet_ptr_t& start, int horizon_count, bool filter_out_lane_changes) const;

    /*! Determine the set of routes(one route is a vector of consecutive lanelets) from which \a start is reachable and which are possible to follow from the \a start lanelet in the graph of lanelets.
     *  \param start The lanelet to start
     *  \param horizon_previous The number of lanelets previous of \a start
     *  \param horizon_following The number of lanelets following \a start
     *  \param filter_out_lane_changes Used for following_set() function call
     *  \return A set of routes containing \a start with a certain amount of lanelets before and after (actually the return value consists of a std::pair containing the route length and the route itself)
     *  @note This may not be the most performant solution! Think about solving this with the Graph directly. Don't use this with too long distances
     * (depending on the length of the lanelets) if you need fast results
     */
    std::set<std::pair<double,std::vector<lanelet_ptr_t> > > possible_routes_set(const lanelet_ptr_t& start, int horizon_previous, int horizon_following, bool filter_out_lane_changes) const;


    /*! Determine the set of lanelets that contains all lanelets beside the given lanelet.
     *  \param start The lanelet to start
     *  \return A set of lanelets that are beside \a start
     */
    std::set<lanelet_ptr_t> beside_set(const lanelet_ptr_t& start) const;


    /*!
     * \brief corridor Determines a corridor from start to end. A corridor contains of all lanelets along the shortest path that can be used for driving.
     * \param start The start lanelet of the corridor
     * \param end The end lanelet of the corridor
     * \return The corridor
     */
    corridor_ptr_t corridor(const lanelet_ptr_t& start, const lanelet_ptr_t &end) const;


    /*! Determine the lanelet which, when interpreted as a polygon's area,
     *  is covering the given point's coordinates.
     * \param query The point to check
     * \return A set of lanelets that cover the area containing the queried coordinates.
     */
    std::set<lanelet_ptr_t> lanelet_by_covered_point(const point_with_id_t& query) const;

    /*!
     * Matches the given position on the closest lanelet which is not farther away than max_distance
     * \param source Source position to find the closest lanelet to
     * \param lanelet When not null, this lanelet is first tested if it contains the given position. When
     * a suitable lanelet is found, it is stored here as a result
     * \param max_distance Maximum distance between the input position and a lanelet. If no lanelet within
     * maximum_distance exists, the resulting lanelet is null
     * \param heading When not null, the (driving) direction of the result position is stored here as
     * a rad value, range -pi..pi with 0 indicating east in counter clockwise direction
     * \param check_heading When true, check whether \a vehicle_heading fits to lanelet heading before returning
     * \param vehicle_heading The actual heading in rad (same definition as \a heading) of the vehicle which is at \a source
     * \param threshold The max allowed angle in rad between the \a vehicle_heading and \a heading
     * \return The source position projected on the closest point on the center polyline of the closest lanelet
     * if a suitable lanelet is found, and the source position otherwise
     */
    point_with_id_t map_matching(const point_with_id_t &source, lanelet_ptr_t &lanelet, double max_distance, double *heading=0, bool check_heading=false, double vehicle_heading=0, double threshold=M_PI/4) const;

    /*!
     * Calculates a score for the match between a lanelet and a vehicle given the metric and orientation distance based on gaussian-like distributions
     * \param metric_distance Distance in meters between the lanelet and the vehicle
     * \param orientation_distance Scalarproduct between normalized lanelet heading vector and vehicle velocity vector (value between -1 and 1)
     * \return The score  0 < score < 1 (0 = no fit, 1 = perfect fit)
     */
    double score_lanelet(double metric_distance, double orientation_distance) const;

    /*!
     * Matches the given position on the best fitting lanelets which are not farther away than max_distance, considering the distance and heading
     * \param source Source position to find the closest lanelet to
     * \param lanelet_list When suitable lanelets are found, they are stored here as a result with there corresponding probabilities
     * \param max_distance Maximum distance between the input position and a lanelet. If no lanelet within
     * maximum_distance exists, the resulting lanelet is null
     * \param vehicle_heading The actual heading in rad (range -pi..pi with 0 indicating east in counter clockwise direction) of the vehicle which is at \a source
     * \return The source position projected on the closest point on the center polyline of the closest lanelet
     * if a suitable lanelet is found, and the source position otherwise
     */
    void map_matching_probabilistic(const point_with_id_t &source, std::vector<std::pair<double,lanelet_ptr_t> > &lanelet_list, double max_distance, boost::tuple< double, double > vehicle_velocity, double threshold=0.1) const;

    //! \todo replace the version above with this one, since it would be no extra effort if we would place the result directly into the multimap.
    void map_matching_probabilistic(const point_with_id_t &source, LaneletByProbabilityMap &lanelet_map, double max_distance, boost::tuple< double, double > vehicle_velocity, double threshold=0.1) const;

    const lanelet_ptr_t& lanelet_by_id( int64_t id ) const;

    const Graph& graph() const;

    //! Read access to all of the map's lanelets
    const std::vector< lanelet_ptr_t >& lanelets() const;

    //! Read access to all of the map's parking spaces
    const std::vector< parking_space_ptr_t >& parking_spaces() const;

    //! Add parking space to the map parking spaces
    bool add_parking_space(parking_space_ptr_t parking_space);

    //! Read access to all of the map's event region
    const std::vector< event_region_ptr_t >& event_regions() const;

    //! Read access to all of the map's traffic lights
    const std::vector< traffic_light_ptr_t >& traffic_lights() const;

    //! returns the minimum node id-1
    int64_t get_minimum_node_id();

    /*! Calculate the (metric) bounding box of the map considering all contained lanelets.
     *  \param gnss_reference_point The reference point for conversion to metric coordinate system
     *  \param lower_left The lower values in x and y dimension of the bounding box rectangle
     *  \param upper_right The upper values in x and y dimension of the bounding box rectangle
     */
    void bounding_box(const point_with_id_t& gnss_reference_point, point_xy_t& lower_left, point_xy_t& upper_right) const;

    /*!
     * \brief toXml Create a TinyXML document describing the given lanelets.
     * \return A XML document.
     */
    TiXmlDocument toXml();



private:
    void init();
    LLTree _lanelet_tree;

    const std::vector< lanelet_ptr_t > _lanelets;
    std::vector< parking_space_ptr_t > _parking_spaces;
    const std::vector< event_region_ptr_t > _event_regions;
    const std::vector< traffic_light_ptr_t > _traffic_lights;
    int64_t _minimum_node_id;

    LLet::Graph _graph;



    int64_t vertex_id_by_lanelet( const lanelet_ptr_t& lanelet) const;
};

typedef boost::shared_ptr< LaneletMap > lanelet_map_ptr_t;

}
