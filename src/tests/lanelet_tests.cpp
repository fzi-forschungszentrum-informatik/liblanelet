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


#include <gtest/gtest.h>
#include <algorithm>

#include <liblanelet/llet_xml.hpp>
#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>
#include <liblanelet/RegulatoryElement.hpp>
#include <liblanelet/prettyprint.hpp>

#include <boost/graph/adjacency_list.hpp>

namespace {

using namespace LLet;

class LaneletTest : public ::testing::Test {
protected:

    LaneletTest() : map_fn("share/osm/sample.osm"), lanelets( parse_xml(map_fn) ), _id(22064)
    {

    }

    const std::string map_fn;

    std::vector< lanelet_ptr_t > lanelets;
    const int32_t _id;

};

TEST_F(LaneletTest, ConstructionOfLaneletsIsCorrect) {

    ASSERT_EQ(lanelets.size(), 6);

    auto pos = std::find_if(lanelets.cbegin(), lanelets.cend(), [this] (const lanelet_ptr_t& ll) { return ll->id() == this->_id; });
    ASSERT_NE(pos, lanelets.cend());

    lanelet_ptr_t llet = *pos;
    ASSERT_EQ(llet->id(), _id);
}

TEST_F(LaneletTest, PointGeometryFunctions)
{
    typedef boost::tuple< double, double > vec_t;

    vec_t a(boost::make_tuple(1, 0)), b(boost::make_tuple(0, 1));
    vec_t c(boost::make_tuple(0, -1));

    ASSERT_DOUBLE_EQ(abs(a), 1.0);
    ASSERT_DOUBLE_EQ(abs(b), 1.0);

    double pi_2 = M_PI/2.0;

    ASSERT_DOUBLE_EQ(angle(a, b), pi_2);
    ASSERT_DOUBLE_EQ(angle(a, c), -pi_2);
}

TEST_F(LaneletTest, MapInsertionAndRetrieval)
{
    boost::shared_ptr< LaneletMap > map;
    ASSERT_NO_THROW( map = boost::make_shared<LaneletMap>(lanelets) );

    lanelet_ptr_t the_ll;
    ASSERT_NO_THROW(the_ll = map->lanelet_by_id(_id));
    ASSERT_THROW(map->lanelet_by_id(-1000000), std::runtime_error);
    ASSERT_NE(the_ll, nullptr);

    BoundingBox world( boost::make_tuple(-180, -180, 180, 180) );
    BoundingBox far_off( boost::make_tuple(-180, -180, -170, -170) );

    ASSERT_EQ( map->query(world).size(), lanelets.size() );
    ASSERT_EQ( map->query(far_off).size(), 0 );
}

TEST_F(LaneletTest, NodeAccessFunctionsAreConsistent)
{    
    LaneletMap map( map_fn );

    lanelet_ptr_t llet = map.lanelet_by_id( _id );
    std::vector< point_with_id_t > nodes_left(llet->nodes(LEFT)), nodes_right(llet->nodes(RIGHT));

    ASSERT_EQ( nodes_left.size(), 10 );
    ASSERT_EQ( nodes_right.size(), 8 );

    ASSERT_EQ( llet->node_at(LEFT, -1), llet->nodes(LEFT).back() );
    ASSERT_EQ( llet->node_at(LEFT, -nodes_left.size()), llet->nodes(LEFT).front() );

    ASSERT_THROW( llet->node_at(LEFT, -nodes_left.size() - 1), std::runtime_error );
}

TEST_F(LaneletTest, ResortingOfWaysWorksCorrectly)
{
    LaneletMap map( map_fn );
    lanelet_ptr_t llet = map.lanelet_by_id( _id );

    std::vector< point_with_id_t > nodes_right(llet->nodes(RIGHT)), nodes_left(llet->nodes(LEFT));

    point_with_id_t first_right(nodes_right.front()), last_right(nodes_right.back());
    point_with_id_t first_left(nodes_left.front()), last_left(nodes_left.back());

    double dist_left  = dist(first_left, last_left);
    double dist_right = dist(first_right, last_right);

    ASSERT_NEAR(dist_right, 16.6, 0.1);
    ASSERT_NEAR(dist_left, 23.5, 0.1);

    ASSERT_EQ( boost::get< ID >(first_left), 581367 );
    ASSERT_EQ( boost::get< ID >(last_left), 581380 );

    // this one is pretty difficult to test, test may succeed incidental
    ASSERT_EQ( boost::get< ID >(first_right), 581375);
    ASSERT_EQ( boost::get< ID >(last_right), 581381);

    // test another one
    llet = map.lanelet_by_id(22060);
    ASSERT_EQ(boost::get<ID>(llet->node_at(LEFT, 0)), 581342);
    ASSERT_EQ(boost::get<ID>(llet->node_at(LEFT, -1)), 581380);

    llet = map.lanelet_by_id(22059);
    ASSERT_EQ(boost::get<ID>(llet->node_at(LEFT, 0)), 581380);
    ASSERT_EQ(boost::get<ID>(llet->node_at(LEFT, -1)), 581393);
}



TEST_F(LaneletTest, AttributeStuffWorks)
{
    LaneletMap map(map_fn);

    lanelet_ptr_t the_ll = map.lanelet_by_id(_id);

    ASSERT_EQ(the_ll->attributes().size(), 3);

    ASSERT_EQ(the_ll->attribute("type").as_string(), "lanelet");
    ASSERT_DOUBLE_EQ(the_ll->attribute("speedlimit").as_double(), 40);

    ASSERT_THROW(the_ll->attribute("type").as_double(), std::runtime_error);

}

TEST_F(LaneletTest, RegulatoryElementsAreParsedCorrectly)
{
    using namespace LLet;
    LaneletMap map( map_fn );
    lanelet_ptr_t the_ll = map.lanelet_by_id(22061);
    ASSERT_EQ( the_ll->regulatory_elements().size(), 1 );
    regulatory_element_ptr_t the_element = the_ll->regulatory_elements().front();

    // now: use the element and check its properties.
    ASSERT_EQ( the_element->attribute("maneuver").as_string(), "traffic_light" );

    auto all_refs = the_element->members("ref");
    ASSERT_EQ(all_refs.size(), 2); // should be two signal lights

    for( const auto& ref: all_refs )
    {
        ASSERT_NO_THROW(boost::get< point_with_id_t >(ref));
        ASSERT_ANY_THROW(boost::get< lanelet_ptr_t >(ref));
        ASSERT_ANY_THROW(boost::get< strip_ptr_t >(ref));
    }

    auto all_lines = the_element->members("stop_line");
    ASSERT_EQ(all_lines.size(), 1);

    const auto& the_line = all_lines.front();

    ASSERT_ANY_THROW(boost::get<lanelet_ptr_t>(the_line));
    ASSERT_NO_THROW(boost::get<strip_ptr_t>(the_line));

}

TEST_F(LaneletTest, GraphSetupWorks)
{
    using namespace LLet;
    LaneletMap map( map_fn );

    ASSERT_EQ(boost::num_vertices(map.graph()), 6);
    ASSERT_EQ(boost::num_edges(map.graph()), 5);

}

TEST_F(LaneletTest, RoutingWorks)
{
    using namespace LLet;
    LaneletMap map( map_fn );

    auto start = map.lanelet_by_id(22063);
    auto dest = map.lanelet_by_id(22061);

    std::vector< lanelet_ptr_t > shortest_path;
    ASSERT_NO_THROW(shortest_path = map.shortest_path(start, dest) );
    ASSERT_EQ(shortest_path.size(), 3);
    ASSERT_NO_THROW(shortest_path = map.shortest_path(start, map.lanelet_by_id(22060)));
    ASSERT_EQ(shortest_path.size(), 0);
}

TEST_F(LaneletTest, LaneletByCoveredPointWorks)
{
  using namespace LLet;
  LaneletMap map( map_fn );
  const point_with_id_t query = boost::make_tuple(49.01587449154474, 8.494360872311582, int64_t(-1));

  std::set<lanelet_ptr_t> lset = map.lanelet_by_covered_point(query);
  ASSERT_EQ(lset.size(), 2);

  for (std::set<lanelet_ptr_t>::const_iterator it=lset.begin();
       it != lset.end(); ++it)
  {
    ASSERT_EQ((*it)->covers_point(query), true);
  }
}

TEST_F(LaneletTest, CalculateCenterLineStripWorks)
{
  using namespace LLet;
  using boost::get;

  LaneletMap map( map_fn );
  const point_with_id_t query = boost::make_tuple(49.01587449154474, 8.494360872311582, int64_t(-1));

  std::set<lanelet_ptr_t> lset = map.lanelet_by_covered_point(query);
  ASSERT_NE(lset.size(), 0); // precondition for the rest to work

  const std::size_t nr_points_left   = get<LEFT>((*lset.begin())->bounds())->pts().size();
  const std::size_t nr_points_right  = get<RIGHT>((*lset.begin())->bounds())->pts().size();
  const std::size_t nr_points_center = get<CENTER>((*lset.begin())->bounds())->pts().size();

  const std::size_t max = std::max(nr_points_left, nr_points_right);

  ASSERT_EQ(max, nr_points_center);
  // todo: a more qualitative test would be nice..
}

TEST_F(LaneletTest, SignedDistanceToLineStipWorks)
{
  using namespace LLet;
  using boost::get;

  LaneletMap map( map_fn );
  const point_with_id_t query = boost::make_tuple(49.01587449154474, 8.494360872311582, int64_t(-1));

  const std::set<lanelet_ptr_t> lset = map.lanelet_by_covered_point(query);
  ASSERT_NE(lset.size(), 0); // precondition for the rest to work

  const strip_ptr_t left  = get<LEFT>((*lset.begin())->bounds());
  const strip_ptr_t right = get<RIGHT>((*lset.begin())->bounds());

  ASSERT_LT(left->signed_distance(query), 0);
  ASSERT_GT(right->signed_distance(query), 0);
  // todo: a more qualitative test would be nice..
}


}  // namespace

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

