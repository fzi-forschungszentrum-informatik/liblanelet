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

#include "llet_xml.hpp"
#include "LineStrip.hpp"
#include "ParkingSpace.hpp"
#include "EventRegion.hpp"
#include "TrafficLight.hpp"
#include "RegulatoryElement.hpp"

#include "Exceptions.h"

#include <pugixml.hpp>
#include "prettyprint.hpp"

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <map>
#include <iostream>
#include <cmath>
#include <cassert>
#include <list>

#define LLET_NAME corridor
#define REG_ELEM_NAME traffic_element

using namespace LLet;

namespace
{

struct TagWalker : public pugi::xml_tree_walker
{
    AttributeMap attributes;

    virtual bool for_each(pugi::xml_node& node)
    {
        if( node.name() == pugi::string_t("tag") )
        {
            std::string k = node.attribute("k").value();
            std::string v = node.attribute("v").value();
            attributes[k] = v;
        }

        return true;
    }
};

struct WayTreeWalker : public pugi::xml_tree_walker
{
    WayTreeWalker(std::map< int64_t, point_with_id_t >& p_b_i, OSMLineStrip& strp) : points_by_id(p_b_i), linestrip(strp)
    {

    }

    std::map< int64_t, point_with_id_t >& points_by_id;
    OSMLineStrip& linestrip;

    virtual bool for_each(pugi::xml_node& node)
    {
        if( node.name() == pugi::string_t("tag") )
        {
            std::string k = node.attribute("k").value();
            std::string v = node.attribute("v").value();
            linestrip._attributes[k] = v;
        }

        else if(  node.name() == pugi::string_t("nd") )
        {
            int64_t node_id = boost::lexical_cast< int64_t >(node.attribute("ref").value());
            linestrip._pts.push_back( points_by_id[node_id] );
        }

        else
        {
            std::cout << node.name() << std::endl;
            std::cout << "doing nothing" << std::endl;
        }

        return true;
    }
};

struct WayTreeParkingSpaceWalker : public pugi::xml_tree_walker
{
  WayTreeParkingSpaceWalker(std::map< int64_t, point_with_id_t >& _p_b_i,
                            std::vector<point_with_id_t>& _data_points,
                            ParkingSpace::ParkingDirection& _direction)
    : p_b_i(_p_b_i), data_points(_data_points), direction(_direction)
  {
  }

  std::map< int64_t, point_with_id_t >& p_b_i;
  std::vector<point_with_id_t>& data_points;
  ParkingSpace::ParkingDirection& direction;

  virtual bool for_each(pugi::xml_node& node)
  {
    if( node.name() == pugi::string_t("tag") )
    {
      std::string key = node.attribute("k").value();
      std::string value = node.attribute("v").value();

      if(key == "parking_direction")
      {
        if (value == "both")
        {
          direction = ParkingSpace::BOTH;
        }
        else if (value == "forward")
        {
          direction = ParkingSpace::FORWARD;
        }
        else if (value == "backward")
        {
          direction = ParkingSpace::BACKWARD;
        }
      }
    }
    else if( node.name() == pugi::string_t("nd") )
    {
      int64_t node_id = boost::lexical_cast< int64_t >(node.attribute("ref").value());
      data_points.push_back(p_b_i[node_id]);
    }
    else
    {
      std::cout << "Node with id: "  <<  node.name() << ", action: " << "doing nothing" << std::endl;
    }

    return true;
  }
};


struct WayTreeEventRegionWalker : public pugi::xml_tree_walker
{
  WayTreeEventRegionWalker(std::map< int64_t, point_with_id_t >& _p_b_i,
                           std::vector<point_with_id_t>& _data_points,
                           EventRegion::Type& _type)
    : points_by_id(_p_b_i), data_points(_data_points), region_type(_type)
  {
    data_points.clear();
    region_type = EventRegion::UNKNOWN;
  }

  std::map< int64_t, point_with_id_t >& points_by_id;
  std::vector<point_with_id_t>& data_points;
  EventRegion::Type& region_type;

  virtual bool for_each(pugi::xml_node& node)
  {
    if( node.name() == pugi::string_t("tag") )
    {
      std::string key = node.attribute("k").value();
      std::string value = node.attribute("v").value();

      if (key == "event_region")
      {
        if (value == "intersection")
        {
          region_type = EventRegion::INTERSECTION;
        }
        else if (value == "parking")
        {
          region_type = EventRegion::PARKING;
        }
        else if (value == "overtaking")
        {
          region_type = EventRegion::OVERTAKING;
        }
        else if (value == "speedlimit")
        {
          region_type = EventRegion::SPEEDLIMIT;
        }
        else
        {
          region_type = EventRegion::UNKNOWN;
        }
      }
    }
    else if( node.name() == pugi::string_t("nd") )
    {
      int64_t node_id = boost::lexical_cast< int64_t >(node.attribute("ref").value());
      data_points.push_back(points_by_id[node_id]);
    }
    else
    {
      std::cout << node.name() << "doing nothing" << std::endl;
    }

    return true;
  }
};

struct WayTreeTrafficLightWalker : public pugi::xml_tree_walker
{
  WayTreeTrafficLightWalker(std::map< int64_t, point_with_id_t >& _p_b_i,
                        std::vector<point_with_id_t>& _data_points,
                        TrafficLight::Type& _type,
                        int64_t& _intersection_id,
                        int64_t& _signalgroup_id)
  : points_by_id(_p_b_i)
  , data_points(_data_points)
  , traffic_light_type(_type)
  , intersection_id(_intersection_id)
  , signalgroup_id(_signalgroup_id)
  {
    data_points.clear();
  }

  std::map< int64_t, point_with_id_t >& points_by_id;
  std::vector<point_with_id_t>& data_points;
  TrafficLight::Type& traffic_light_type;
  int64_t& intersection_id;
  int64_t& signalgroup_id;

  virtual bool for_each(pugi::xml_node& node)
  {
    if( node.name() == pugi::string_t("tag") )
    {
      std::string key = node.attribute("k").value();
      std::string value = node.attribute("v").value();

      if (key == "RoadSign:Type")
      {
        if (value == "DE/1.000.001")
        {
          traffic_light_type = TrafficLight::ALL;
        }
        else if(value == "DE/1.000.011-10")
        {
          traffic_light_type = TrafficLight::LEFT_ONLY;
        }
        else if(value == "DE/1.000.011-20")
        {
          traffic_light_type = TrafficLight::RIGHT_ONLY;
        }
        else if(value == "DE/1.000.011-30")
        {
          traffic_light_type = TrafficLight::STRAIGHT_ONLY;
        }
        else if(value == "DE/1.000.011-40")
        {
          traffic_light_type = TrafficLight::STRAIGHT_LEFT_ONLY;
        }
        else if(value == "DE/1.000.011-50")
        {
          traffic_light_type = TrafficLight::STRAIGHT_RIGHT_ONLY;
        }
      }
      else if (key == "intersection_id")
      {
        intersection_id = node.attribute("v").as_llong();
      }
      else if (key == "signalgroup_id")
      {
        signalgroup_id = node.attribute("v").as_llong();
      }
    }
    else if( node.name() == pugi::string_t("nd") )
    {
      int64_t node_id = boost::lexical_cast< int64_t >(node.attribute("ref").value());
      data_points.push_back(points_by_id[node_id]);
    }

    return true;
  }
};

struct XMLParser
{
    std::map< int64_t, point_with_id_t > points_by_id;
    std::map< int64_t, strip_ptr_t > linestrips_by_id;
    std::map< int64_t, lanelet_ptr_t > lanelets_by_id;
    std::map< int64_t, parking_space_ptr_t > parking_spaces_by_id;
    std::map< int64_t, event_region_ptr_t > event_regions_by_id;
    std::map< int64_t, traffic_light_ptr_t > traffic_lights_by_id;
    std::map< int64_t, regulatory_element_ptr_t > regulatory_elements_by_id;

    pugi::xml_document& doc;
    bool m_ignore_consistency_failures;

    void parse_nodes();
    void parse_linestrips();
    void parse_lanelets();
    void parse_parking_spaces();
    void parse_event_regions();
    void parse_traffic_lights();
    void parse_regulatory_elements_and_assign_to_lanelets();

    std::vector< lanelet_ptr_t > parsed_lanelets() const;
    std::vector< parking_space_ptr_t > parsed_parking_spaces() const;
    std::vector< event_region_ptr_t > parsed_event_regions() const;
    std::vector< traffic_light_ptr_t > parsed_traffic_lights() const;

    double phi(std::map<std::string, strip_ptr_t> &bounds);
    int64_t get_minimum_node_id();

    XMLParser( pugi::xml_document& doc, const bool ignore_consistency_failures=false) : doc(doc), m_ignore_consistency_failures(ignore_consistency_failures)
    {
        parse_nodes();
        parse_linestrips();
        parse_lanelets();
        parse_parking_spaces();
        parse_event_regions();
        parse_traffic_lights();
        parse_regulatory_elements_and_assign_to_lanelets();
    }
};

std::vector< lanelet_ptr_t > XMLParser::parsed_lanelets() const
{
    std::vector< lanelet_ptr_t > result;
    typedef std::map< int64_t, lanelet_ptr_t >::value_type value_type;
    BOOST_FOREACH( const value_type& kv, lanelets_by_id )
    {
        result.push_back( kv.second );
    }

    return result;
}

std::vector< parking_space_ptr_t > XMLParser::parsed_parking_spaces() const
{
    std::vector< parking_space_ptr_t> result;
    typedef std::map< int64_t, parking_space_ptr_t >::value_type value_type;
    BOOST_FOREACH( const value_type& kv, parking_spaces_by_id )
    {
        result.push_back( kv.second );
    }

    return result;
}


std::vector< event_region_ptr_t > XMLParser::parsed_event_regions() const
{
    std::vector< event_region_ptr_t > result;
    typedef std::map< int64_t, event_region_ptr_t >::value_type value_type;
    BOOST_FOREACH( const value_type& kv, event_regions_by_id )
    {
        result.push_back( kv.second );
    }

    return result;
}

std::vector< traffic_light_ptr_t > XMLParser::parsed_traffic_lights() const
{
  std::vector< traffic_light_ptr_t > result;
  typedef std::map< int64_t, traffic_light_ptr_t >::value_type value_type;
  BOOST_FOREACH( const value_type& kv, traffic_lights_by_id )
  {
    result.push_back( kv.second );
  }

  return result;
}

void XMLParser::parse_nodes()
{
    BOOST_FOREACH ( pugi::xpath_node node, doc.select_nodes("//node[@lat and @lon and @id]") )
    {
        double lat = node.node().attribute("lat").as_double();
        double lon = node.node().attribute("lon").as_double();
        int64_t id = node.node().attribute("id").as_llong();
        point_with_id_t point = boost::make_tuple(lat, lon, id);
        TagWalker tagwalker;
        node.node().traverse(tagwalker);
        point.attributes() = tagwalker.attributes;
        points_by_id[id] = point;
    }
}

void XMLParser::parse_linestrips()
{
    BOOST_FOREACH( pugi::xpath_node way, doc.select_nodes("//way[@id]") )
    {
        boost::shared_ptr<OSMLineStrip> new_linestrip(new OSMLineStrip());
        WayTreeWalker walker(points_by_id, *new_linestrip);
        way.node().traverse(walker);
        linestrips_by_id[way.node().attribute("id").as_llong()] = new_linestrip;
    }
}

// angle between (a, b) should be somewhere near +pi/2.
double XMLParser::phi(std::map< std::string, strip_ptr_t >& bounds)
{
        boost::tuple< double, double > vec_b = vec(bounds["left"]->pts()[0], bounds["left"]->pts()[1]);
        boost::tuple< double, double > vec_a = vec(bounds["left"]->pts()[0], bounds["right"]->pts()[0]);
        return angle( vec_a, vec_b);
}

//returns the smallest id (ids are negative)
int64_t XMLParser::get_minimum_node_id(){
  int64_t min = 0;

  std::pair<int64_t, point_with_id_t> point;
  BOOST_FOREACH (point, points_by_id )
  {
    if(point.first < min)
      min = point.first;
  }

  std::pair<int64_t, strip_ptr_t> strip;
  BOOST_FOREACH(strip, linestrips_by_id)
  {
    if(strip.first < min)
      min = strip.first;
  }

  BOOST_FOREACH(lanelet_ptr_t lanelet, parsed_lanelets())
  {
    if(lanelet->id() < min)
      min = lanelet->id();
  }

  BOOST_FOREACH(parking_space_ptr_t parking_space, parsed_parking_spaces())
  {
    if(parking_space->getId() < min)
      min = parking_space->getId();
  }

  BOOST_FOREACH(event_region_ptr_t event_region, parsed_event_regions())
  {
    if(event_region->getId() < min)
      min = event_region->getId();
  }

  BOOST_FOREACH(traffic_light_ptr_t traffic_light, parsed_traffic_lights())
  {
    if(traffic_light->getId() < min)
      min = traffic_light->getId();
  }

  std::pair<int64_t, regulatory_element_ptr_t> regulatory_element;
  BOOST_FOREACH(regulatory_element, regulatory_elements_by_id)
  {
    if(regulatory_element.first <min)
      min = regulatory_element.first;
  }

  return min;
}

void XMLParser::parse_lanelets()
{
    std::list<std::string> roles;
    roles.push_back("left");
    roles.push_back("right");
    BOOST_FOREACH( pugi::xpath_node relation, doc.select_nodes("//relation/tag[@v='lanelet' and @k='type']/..") )
    {
        std::map< std::string, strip_ptr_t > bounds;
        int64_t id = relation.node().attribute("id").as_llong();

        TagWalker tagwalker;
        relation.node().traverse(tagwalker);

        BOOST_FOREACH(std::string role, roles)
        {
            std::vector< strip_ptr_t > line_strips_for_this_bound;
            BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes((boost::format("member[@type='way' and @role='%s']") % role).str().c_str()) )
            {
                int64_t ref_id = member.node().attribute("ref").as_llong();
                line_strips_for_this_bound.push_back( linestrips_by_id[ref_id] );
            }

            if (line_strips_for_this_bound.empty())
            {
                std::stringstream stream;
                stream << "Lanelet " << id << " has no bounds for the " << role << " side.";
                if (m_ignore_consistency_failures)
                {
                  std::cout << "Warning: Lanelet consistency check failed: " << stream.str() << std::endl;

                  //tag lanelet:
                  AttributeValue attr(stream.str());
                  tagwalker.attributes.insert(std::pair<std::string, AttributeValue>("lanelet_checker_problem", attr));
                }
                else
                {
                  throw LaneletConsistencyError(stream.str());
                }
            }
            else
            {
                try {
                  CompoundLineStrip* comp = new CompoundLineStrip(line_strips_for_this_bound);
                  bounds[role] = boost::shared_ptr< LineStrip >(comp);
                }
                catch (const std::runtime_error &) {
                  std::stringstream stream;
                  stream << "Lanelet " << id << " is not fully connected.";
                  if (m_ignore_consistency_failures)
                  {
                    std::cout << "Warning: Lanelet consistency check failed: " << stream.str() << std::endl;

                    //tag lanelet:
                    AttributeValue attr(stream.str());
                    tagwalker.attributes.insert(std::pair<std::string, AttributeValue>("lanelet_checker_problem", attr));
                  }
                  else
                  {
                    throw LaneletConsistencyError(stream.str());
                  }
                }
            }


        }
        //if size of bounds is less than 2, "left" or "right" is missing -> continue with next lanelet
        if (bounds.size() < 2)
        {
            continue;
        }
        const std::vector< point_with_id_t >& pts_left = bounds["left"]->pts();
        const std::vector< point_with_id_t >& pts_right = bounds["right"]->pts();

        if( dist(pts_left.front(), pts_right.front()) > dist(pts_left.front(), pts_right.back()) )
        {
            bounds["left"] = boost::make_shared< ReversedLineStrip >(bounds["left"]);
        }

        //  std::cout << "angle is " << phi * 180 / M_PI << std::endl;
        double pi_2 = M_PI * 0.5;

        if( ! inrange(phi(bounds),  0.5 * pi_2, 1.5 * pi_2 ) )
        {
            bounds["left"] = boost::make_shared< ReversedLineStrip >(bounds["left"]);
            bounds["right"] = boost::make_shared< ReversedLineStrip >(bounds["right"]);
        }

        if ( !inrange(phi(bounds),  0.5 * pi_2, 1.5 * pi_2 ) )
        {
          std::stringstream stream;
          stream << 180.0 * 0.5 * pi_2 / M_PI;
          stream << " < phi=" << 180.0 * phi(bounds) / M_PI;
          stream << " < " << 180.0 * 1.5 * pi_2 / M_PI;
          stream << " does not hold, check the strips of lanelet " << id;
          if (m_ignore_consistency_failures)
          {
            std::cout << "Warning: Lanelet consistency check failed: " << stream.str() << std::endl;

            //tag lanelet:
            AttributeValue attr(stream.str());
            tagwalker.attributes.insert(std::pair<std::string, AttributeValue>("lanelet_checker_problem", attr));
          }
          else
          {
            throw LaneletConsistencyError(stream.str());
          }
        }

        if(bounds["left"]->pts().size() < 2 || bounds["right"]->pts().size() < 2)
        {
          if (m_ignore_consistency_failures)
          {
            std::cout << "Warning: Lanelet(" << id << ") with less than two points on linestrip was read, but removed."
                      << std::endl;
          }
          else
          {
            std::stringstream stream;
            stream << "Lanelet with less than two points on linestrip was read.";
            throw LaneletConsistencyError(stream.str());
          }
        }
        else
        {
          //Create the lanelet with the points from the left and right linestrip.
          boost::shared_ptr<Lanelet> new_lanelet = boost::make_shared< Lanelet >(id, bounds["left"], bounds["right"]);

          new_lanelet->attributes() = tagwalker.attributes;
          lanelets_by_id[id] = new_lanelet;
        }
    }
    BOOST_FOREACH( pugi::xpath_node relation, doc.select_nodes("//relation/tag[@v='lanelet' and @k='type']/..") )
    {
      int32_t id = relation.node().attribute("id").as_int();
      if (lanelets_by_id.count(id) > 0)
      {
        boost::shared_ptr<Lanelet> lanelet = lanelets_by_id[id];
        BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes("member[@type='relation']") )
        {
          int32_t ref_id = member.node().attribute("ref").as_int();
          std::string role = member.node().attribute("role").as_string();
          if (lanelets_by_id.count(ref_id) > 0)
          {
            lanelet->add_related_lanelet(role, lanelets_by_id[ref_id]);
          }
        }
      }
    }
}

void XMLParser::parse_parking_spaces()
{
  // Create a parking space for each amenity with value 'parking_space'
  BOOST_FOREACH( pugi::xpath_node way, doc.select_nodes("//way/tag[@v='parking_space' and @k='amenity']/..") )
  {
    std::vector<point_with_id_t> data_points;
    ParkingSpace::ParkingDirection parking_direction;

    WayTreeParkingSpaceWalker walker(points_by_id, data_points, parking_direction);
    way.node().traverse(walker);

    int64_t parking_space_id =  way.node().attribute("id").as_llong();

    if(data_points.size() == ParkingSpace::numberOfExpectedPoints())
    {
      boost::shared_ptr<ParkingSpace> new_parking_space(new ParkingSpace(data_points, parking_direction));
      new_parking_space->setId(parking_space_id);

      parking_spaces_by_id[parking_space_id] = new_parking_space;
    }
    else
    {
      std::stringstream stream;
      stream << "Parsing Error: parking space is not valid with id: " << parking_space_id;
      throw std::runtime_error(stream.str());
    }
  }

  // Add related lanelets to each parking space
  BOOST_FOREACH( pugi::xpath_node relation, doc.select_nodes("//relation/tag[@v='accessible' and @k='type']/..") )
  {
    BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes("member[@type='way' and @role='parking_space']"))
    {
      int64_t parking_space_id = member.node().attribute("ref").as_llong();

      BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes("member[@type='relation' and @role='lanelet']"))
      {
         int64_t lanelet_id = member.node().attribute("ref").as_llong();
         parking_spaces_by_id[parking_space_id]->addRelatedLanelet(lanelet_id);
      }
    }
  }
}

void XMLParser::parse_event_regions()
{
  // Create a event region for each way with value ''
  BOOST_FOREACH( pugi::xpath_node way, doc.select_nodes("//way/tag[@k='event_region']/..") )
  {
    std::vector<point_with_id_t> data_points;
    EventRegion::Type event_type;

    WayTreeEventRegionWalker walker(points_by_id, data_points, event_type);
    way.node().traverse(walker);

    TagWalker tagwalker;
    way.node().traverse(tagwalker);

    int64_t event_region_id =  way.node().attribute("id").as_llong();

    if ((data_points.size() >= 3) && (data_points.front().get<2>() == data_points.back().get<2>()))
    {
      boost::shared_ptr<EventRegion> new_event_region(new EventRegion(data_points));
      new_event_region->setId(event_region_id);
      new_event_region->setType(event_type);
      new_event_region->attributes() = tagwalker.attributes;
      event_regions_by_id[event_region_id] = new_event_region;
    }
    else
    {
      std::stringstream stream;
      stream << "Parsing Error: Event region with id " << event_region_id << " is not valid.";
      throw std::runtime_error(stream.str());
    }
  }
}

void XMLParser::parse_traffic_lights()
{
  // Create a traffic light for each way with value ''
  BOOST_FOREACH( pugi::xpath_node way, doc.select_nodes("//way/tag[@k='RoadSign']/..") )
  {
    std::vector<point_with_id_t> data_points;
    TrafficLight::Type traffic_light_type = TrafficLight::UNKNOWN;
    int64_t intersection_id = 0;
    int64_t signalgroup_id  = 0;

    WayTreeTrafficLightWalker walker(points_by_id, data_points, traffic_light_type, intersection_id, signalgroup_id);
    way.node().traverse(walker);

    // walker has found no traffic light tag
    if (traffic_light_type == TrafficLight::UNKNOWN)
    {
      continue;
    }

    int64_t id =  way.node().attribute("id").as_llong();

    if ((data_points.size() >= 3) && (data_points.front().get<2>() == data_points.back().get<2>()))
    {
      boost::shared_ptr<TrafficLight> new_traffic_light(new TrafficLight(data_points));
      new_traffic_light->setId(id);
      new_traffic_light->setIds(intersection_id, signalgroup_id);
      new_traffic_light->setType(traffic_light_type);
      traffic_lights_by_id[id] = new_traffic_light;
    }
    else
    {
      std::stringstream stream;
      stream << "Parsing Error: Traffic light with id " << id << " is not valid.";
      throw std::runtime_error(stream.str());
    }
  }
}

void XMLParser::parse_regulatory_elements_and_assign_to_lanelets()
{
    BOOST_FOREACH( pugi::xpath_node relation, doc.select_nodes("//relation/tag[@v='regulatory_element' and @k='type']/..") )
    {
        int64_t id = relation.node().attribute("id").as_llong();

        TagWalker tagwalker;
        relation.node().traverse(tagwalker);

        boost::shared_ptr<RegulatoryElement> new_reg_elem = boost::make_shared< RegulatoryElement >(id);
        new_reg_elem->attributes() = tagwalker.attributes;

        std::vector< regulatory_element_member_t > all_members;

        BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes("member[@type='way']") )
        {
            int64_t ref = member.node().attribute("ref").as_llong();
            std::string role = member.node().attribute("role").value();

            // if an event region exists for this way, the event region takes precedence
            if (event_regions_by_id.count(ref) == 0)
            {
              member_variant_t __member = linestrips_by_id[ref];
              assert(boost::get< strip_ptr_t >(__member) != NULL);

              all_members.push_back(std::make_pair(role, __member));
            }
            else
            {
              member_variant_t __member = event_regions_by_id[ref];
              assert(boost::get< event_region_ptr_t >(__member) != NULL);

              all_members.push_back(std::make_pair(role, __member));
            }
        }

        BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes("member[@type='node']") )
        {
            int64_t ref = member.node().attribute("ref").as_llong();
            std::string role = member.node().attribute("role").value();

            member_variant_t __member = points_by_id[ref];
            // not possible to check here for nulpointer ?
            all_members.push_back(std::make_pair(role, __member));
        }

        BOOST_FOREACH( pugi::xpath_node member, relation.node().select_nodes("member[@type='relation']") )
        {
            int64_t ref = member.node().attribute("ref").as_llong();
            std::string role = member.node().attribute("role").value();

            if (lanelets_by_id.count(ref) > 0)
            {
              member_variant_t __member = lanelets_by_id[ref];
              assert(boost::get< lanelet_ptr_t >(__member) != NULL);

              all_members.push_back(std::make_pair(role, __member));
            }
        }

        new_reg_elem->members() = all_members;

        regulatory_elements_by_id[id] = new_reg_elem;
    }

    BOOST_FOREACH( pugi::xpath_node lanelet, doc.select_nodes("//relation/tag[@v='lanelet' and @k='type']/..") )
    {
        int64_t relation_id = lanelet.node().attribute("id").as_llong();
        BOOST_FOREACH( pugi::xpath_node reg_elem_member, lanelet.node().select_nodes("member[@type='relation' and @role='regulatory_element']") )
        {
            int64_t ref = reg_elem_member.node().attribute("ref").as_llong();
            lanelets_by_id[relation_id]->regulatory_elements().push_back( regulatory_elements_by_id[ref] );
        }
    }
}


} // END NAMESPACE

std::vector< lanelet_ptr_t > LLet::parse_xml(const std::string &filename, const bool ignore_consistency_failures)
{
    pugi::xml_document doc;
    pugi::xml_parse_result res = doc.load_file(filename.c_str());
    if (res)
    {
        XMLParser parser( doc, ignore_consistency_failures );
        return parser.parsed_lanelets();
    }
    else
    {
        std::cerr << "Failed to parse " << filename << ": " << res.description() << std::endl;
        return std::vector< lanelet_ptr_t >();
    }
}


std::vector< parking_space_ptr_t > LLet::parse_xml_parking_spaces(const std::string& filename, const bool ignore_consistency_failures)
{
    pugi::xml_document doc;
    pugi::xml_parse_result res = doc.load_file(filename.c_str());
    if (res)
    {
      XMLParser parser( doc, ignore_consistency_failures );
      return parser.parsed_parking_spaces();
    }
    else
    {
        std::cerr << "Failed to parse " << filename << ": " << res.description() << std::endl;
        return std::vector< parking_space_ptr_t >();
    }
}

std::vector< event_region_ptr_t > LLet::parse_xml_event_regions(const std::string& filename, const bool ignore_consistency_failures)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(filename.c_str());
  if (result)
  {
    XMLParser parser( doc, ignore_consistency_failures );
    return parser.parsed_event_regions();
  }
  else
  {
    std::cerr << "Failed to parse " << filename << ": " << result.description() << std::endl;
    return std::vector< event_region_ptr_t >();
  }
}

int64_t LLet::get_minimum_node_id(const std::string& filename, const bool ignore_consistency_failures)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(filename.c_str());
  if (result)
  {
    XMLParser parser( doc, ignore_consistency_failures );
    return parser.get_minimum_node_id();
  }
  else
  {
    std::cerr << "Failed to parse " << filename << ": " << result.description() << std::endl;
    return 0;
  }
}

std::vector< traffic_light_ptr_t > LLet::parse_xml_traffic_lights(const std::string& filename, const bool ignore_consistency_failures)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(filename.c_str());
  if (result)
  {
    XMLParser parser( doc, ignore_consistency_failures );
    return parser.parsed_traffic_lights();
  }
  else
  {
    std::cerr << "Failed to parse " << filename << ": " << result.description() << std::endl;
    return std::vector< traffic_light_ptr_t >();
  }
}
