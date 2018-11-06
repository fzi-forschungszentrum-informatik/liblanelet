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

#include "Lanelet.hpp"
#include "Polygon.hpp"

#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

#include <string>
#include <stdexcept>

using namespace LLet;

Lanelet::Lanelet()
{
}

Lanelet::Lanelet(int64_t id, const strip_ptr_t &left, const strip_ptr_t &right) :
    _id(id),
    _bounds(boost::make_tuple(left, right, strip_ptr_t())),
    m_polygon()
{
  // both cannot be done in init list, _bounds must be setup already
  boost::get<LLet::CENTER>(_bounds) = calculate_center_line_strip();
  m_polygon = boost::shared_ptr<Polygon>(new Polygon(*this));
}

const Lanelet::bounds_container_t& Lanelet::bounds() const
{
    return _bounds;
}

const std::vector<regulatory_element_ptr_t> &Lanelet::regulatory_elements() const
{
    return _regulatory_elements;
}

std::vector<regulatory_element_ptr_t> &Lanelet::regulatory_elements()
{
    return _regulatory_elements;
}

std::vector< boost::shared_ptr<Lanelet> > Lanelet::related_lanelets(const std::string& role)
{
  std::vector< boost::shared_ptr<Lanelet> > result;
  for (std::size_t i = 0; i < _lanelets_relations.size(); i++)
  {
    if (_lanelets_relations[i].second == role)
    {
      boost::shared_ptr<Lanelet> p = _lanelets_relations[i].first.lock();
      if (p)
      {
        result.push_back(p);
      }
    }
  }
  return result;
}

int64_t Lanelet::id() const
{
    return _id;
}

void Lanelet::add_regulatory_element(const regulatory_element_ptr_t &elem)
{
  this->_regulatory_elements.push_back( elem );
}

void Lanelet::add_related_lanelet( const std::string& role, const boost::shared_ptr<Lanelet>& lanelet )
{
  this->_lanelets_relations.push_back(std::make_pair(boost::weak_ptr<Lanelet>(lanelet), role));
}

boost::shared_ptr<Lanelet> Lanelet::reverse() const
{
  return boost::make_shared<Lanelet>(_id,
                                     boost::make_shared<ReversedLineStrip>(boost::get<LLet::RIGHT>(_bounds)),
                                     boost::make_shared<ReversedLineStrip>(boost::get<LLet::LEFT>(_bounds)));
}

bool Lanelet::covers_point(const point_with_id_t& query) const
{
  return m_polygon->pointInsidePolygon(query);
}

double Lanelet::distance_to(const point_with_id_t &point) const
{
  return m_polygon->distanceTo(point);
}

strip_ptr_t Lanelet::calculate_center_line_strip() const
{
  const strip_ptr_t& left  = boost::get<LLet::LEFT>(_bounds);
  const strip_ptr_t& right = boost::get<LLet::RIGHT>(_bounds);

  if ((left->pts().size() == 0) || (right->pts().size() == 0)) // no points -> no center line strip
  {
    std::cerr << "left or right bound has no points, aborting." << std::endl;
    return strip_ptr_t();
  }

  // -- generate cross connections between the points of the linestrips --

  std::vector<double> accumulated_ratios_left  = left->accumulated_length_ratios();
  std::vector<double> accumulated_ratios_right = right->accumulated_length_ratios();

  typedef boost::tuple<point_with_id_t, point_with_id_t> cross_connection_t;
  std::vector<cross_connection_t> cross_connections;

  cross_connection_t cc;

  bool const swap = left->pts().size() < right->pts().size();
  std::vector<double> const & accumulated_ratios_a = swap ? accumulated_ratios_right : accumulated_ratios_left;
  std::vector<double> const & accumulated_ratios_b = swap ? accumulated_ratios_left  : accumulated_ratios_right;
  const strip_ptr_t& strip_a = swap ? right : left;
  const strip_ptr_t& strip_b = swap ? left : right;

  // append the first pair
  cc = boost::make_tuple(strip_a->pts().front(), strip_b->pts().front());
  cross_connections.push_back(cc);
  for (std::size_t i=1; i<accumulated_ratios_a.size(); ++i)
  {
    const std::vector<double>::const_iterator destination = std::lower_bound(accumulated_ratios_b.begin(),
                                                                             accumulated_ratios_b.end(),
                                                                             accumulated_ratios_a[i]);
    std::size_t const dest_index = destination - accumulated_ratios_b.begin();

    cc = boost::make_tuple(strip_a->pts()[i], strip_b->pts()[dest_index]);
    cross_connections.push_back(cc);
  }
  // append the last pair
  cc = boost::make_tuple(strip_a->pts().back(), strip_b->pts().back());
  cross_connections.push_back(cc);

  // generate center line strip points out of the middle of each cross connection
  boost::shared_ptr<OSMLineStrip> center_strip(new OSMLineStrip());
  for (std::size_t i=0; i<cross_connections.size(); ++i)
  {
    point_with_id_t center_point;
    const point_with_id_t& gnss_from = boost::get<0>(cross_connections[i]);
    const point_with_id_t& gnss_to   = boost::get<1>(cross_connections[i]);
    assert(i>0 || gnss_from == *strip_a->pts().begin());
    assert(i>0 || gnss_to == *strip_b->pts().begin());

    point_xy_t relative = vec(gnss_from, gnss_to);
    relative = boost::make_tuple(0.5*boost::get<LLet::X>(relative), 0.5*boost::get<LLet::Y>(relative));
    center_point = from_vec(gnss_from, relative);
    center_strip->_pts.push_back(center_point);
  }

  return center_strip;
}

point_with_id_t Lanelet::project(const point_with_id_t& source, double* angle, std::size_t* index, std::size_t* previous_index, std::size_t* subsequent_index) const
{
  vertex_container_t center_line;
  if (!boost::get<LLet::CENTER>(_bounds))
  {
    const_cast<Lanelet*>(this)->calculate_center_line_strip();
  }
  const std::vector<point_with_id_t> & center_coordinates = boost::get<LLet::CENTER>(_bounds)->pts();
  assert(!center_coordinates.empty());
  point_with_id_t const base = *center_coordinates.begin();
  point_xy_t metric_point = vec(base, source);
  BOOST_FOREACH(const point_with_id_t &point, center_coordinates)
  {
    center_line.push_back(vec(base, point));
  }
  point_xy_t t = projected(metric_point, center_line, angle, index, previous_index, subsequent_index);
  return from_vec(base, t);
}

double Lanelet::distance_along_lanelet(const point_with_id_t &source) const
{
  double distance = 0;
  double angle = 0;
  double* angle_ptr = &angle;
  std::size_t index = 0;
  std::size_t* index_ptr = &index;
  std::size_t previous_index = 0;
  std::size_t* previous_index_ptr = &previous_index;
  std::size_t subsequent_index = 0;
  std::size_t* subsequent_index_ptr = &subsequent_index;
  const LLet::point_with_id_t projection = project(source, angle_ptr, index_ptr, previous_index_ptr, subsequent_index_ptr);
  std::vector<LLet::point_with_id_t> nodes = this->nodes(LLet::CENTER);
  std::size_t count = 0;
  std::vector<LLet::point_with_id_t>::iterator ita = nodes.begin();
  while (ita != nodes.end()-1 && count < previous_index)
  {
    std::vector<LLet::point_with_id_t>::iterator itb = ita+1;
    LLet::point_with_id_t pointa = *ita;
    LLet::point_with_id_t pointb = *itb;
    distance += dist(pointa, pointb);
    ++count;
    ++ita;
  }
  LLet::point_with_id_t pointa = *ita;
  distance += dist(pointa, projection);
  return distance;
}

double Lanelet::point_at_distance_along_lanelet(const point_with_id_t &source, point_with_id_t &result, double distance) const
{
  if (distance_along_lanelet(source) + distance >= length(CENTER))
  {
    result = point_with_id_t();
    return length(CENTER) - distance_along_lanelet(source) - distance;
  }
  else
  {
    double angle = 0;
    double* angle_ptr = &angle;
    std::size_t index = 0;
    std::size_t* index_ptr = &index;
    std::size_t previous_index = 0;
    std::size_t* previous_index_ptr = &previous_index;
    std::size_t subsequent_index = 0;
    std::size_t* subsequent_index_ptr = &subsequent_index;
    const LLet::point_with_id_t projection = project(source, angle_ptr, index_ptr, previous_index_ptr, subsequent_index_ptr);

    double leftover_distance = distance;

    LLet::point_with_id_t point_a = projection;
    for (std::size_t index_b = subsequent_index; index_b < nodes(CENTER).size(); ++index_b)
    {
      LLet::point_with_id_t point_b = node_at(CENTER, index_b);
      double distance_a_to_b = dist(point_a,point_b);
      if (leftover_distance < distance_a_to_b)
      {
        double ratio = leftover_distance / distance_a_to_b;
        boost::get<LLet::LAT>(result) = (1.0 - ratio) * boost::get<LLet::LAT>(point_a) + ratio * boost::get<LLet::LAT>(point_b);
        boost::get<LLet::LON>(result) = (1.0 - ratio) * boost::get<LLet::LON>(point_a) + ratio * boost::get<LLet::LON>(point_b);
        return length(CENTER) - distance_along_lanelet(result);
      }
      else
      {
        leftover_distance -= distance_a_to_b;
        point_a = node_at(CENTER,index_b);
      }
    }
    // The only way to fail to escape the for-loop above this is when numerical inaccuracies
    // prevent the condition to get the last element.
    result = node_at(CENTER, nodes(CENTER).size() - 1);
    return 0.0;
  }
}


double Lanelet::point_at_distance_along_lanelet(point_with_id_t &result, double distance) const
{
  if (distance >= length(CENTER))
  {
    result = point_with_id_t();
    return length(CENTER) - distance;
  }
  else
  {
    LLet::point_with_id_t point_a = node_at(CENTER,0);
    double leftover_distance = distance;
    for (std::size_t index_b = 1; index_b < nodes(CENTER).size(); ++index_b)
    {
      LLet::point_with_id_t point_b = node_at(CENTER, index_b);
      double distance_a_to_b = dist(point_a,point_b);
      if (leftover_distance < distance_a_to_b)
      {
        double ratio = leftover_distance / distance_a_to_b;
        boost::get<LLet::LAT>(result) = (1.0 - ratio) * boost::get<LLet::LAT>(point_a) + ratio * boost::get<LLet::LAT>(point_b);
        boost::get<LLet::LON>(result) = (1.0 - ratio) * boost::get<LLet::LON>(point_a) + ratio * boost::get<LLet::LON>(point_b);
        return length(CENTER) - distance_along_lanelet(result);
      }
      else
      {
        leftover_distance -= distance_a_to_b;
        point_a = node_at(CENTER,index_b);
      }
    }
    // same as point_at_distance_along_lanelet()
    result = node_at(CENTER, nodes(CENTER).size() - 1);
    return 0.0;
  }
}


void Lanelet::bounding_box(const point_with_id_t& gnss_reference_point, point_xy_t& lower_left, point_xy_t& upper_right) const
{
  m_polygon->boundingBox(lower_left, upper_right);
  // convert to the desired coordinate system
  lower_left = vec(gnss_reference_point, from_vec(m_polygon->gnssReferencePoint(), lower_left));
  upper_right = vec(gnss_reference_point, from_vec(m_polygon->gnssReferencePoint(), upper_right));
}

bool Lanelet::intersects(const lanelet_ptr_t other_lanelet)
{
  if (this->id() == other_lanelet->id())
  {
    return true;
  }
  else
  {
    const LLet::Polygon* polygon1 = m_polygon.get();
    const LLet::Polygon* polygon2 = other_lanelet->polygon().get();
    std::vector<LLet::point_with_id_t> nodes1 = nodes(LLet::CENTER);
    std::vector<LLet::point_with_id_t> nodes2 = other_lanelet->nodes(LLet::CENTER);
    for (std::vector<LLet::point_with_id_t>::iterator it = nodes2.begin(); it != nodes2.end(); ++it)
    {
      if (polygon1->pointInsidePolygon(*it))
      {
        return true;
      }
    }
    for (std::vector<LLet::point_with_id_t>::iterator it = nodes1.begin(); it != nodes1.end(); ++it)
    {
      if (polygon2->pointInsidePolygon(*it))
      {
        return true;
      }
    }
    return false;
  }
}


point_with_id_t Lanelet::intersection_point(const lanelet_ptr_t other_lanelet, double& distance_this, double& distance_other)
{
  point_with_id_t intersection;
  if (intersects(other_lanelet))
  {
    if (this->id() != other_lanelet->id())
    {
      std::vector<LLet::point_with_id_t> nodes1 = this->nodes(LLet::CENTER);
      std::vector<LLet::point_with_id_t> nodes2 = other_lanelet->nodes(LLet::CENTER);
      distance_this = 0;

      for (std::vector<LLet::point_with_id_t>::iterator it1a = nodes1.begin(); it1a != nodes1.end()-1; ++it1a)
      {
        std::vector<LLet::point_with_id_t>::iterator it1b = it1a+1;
        LLet::point_with_id_t point1a = *it1a;
        LLet::point_with_id_t point1b = *it1b;
        distance_this += dist(point1a, point1b);

        distance_other = 0;
        for (std::vector<LLet::point_with_id_t>::iterator it2a = nodes2.begin(); it2a != nodes2.end()-1; ++it2a)
        {
          std::vector<LLet::point_with_id_t>::iterator it2b = it2a+1;
          LLet::point_with_id_t point2a = *it2a;
          LLet::point_with_id_t point2b = *it2b;
          distance_other += dist(point2a, point2b);

          double point1a_LAT = boost::get<LLet::LAT>(point1a);
          double point1a_LON = boost::get<LLet::LON>(point1a);
          double point1b_LAT = boost::get<LLet::LAT>(point1b);
          double point1b_LON = boost::get<LLet::LON>(point1b);

          double point2a_LAT = boost::get<LLet::LAT>(point2a);
          double point2a_LON = boost::get<LLet::LON>(point2a);
          double point2b_LAT = boost::get<LLet::LAT>(point2b);
          double point2b_LON = boost::get<LLet::LON>(point2b);

          // check whether points are equal
         if (point1a_LON == point2a_LON && point1a_LAT == point2a_LAT)
         {
           distance_this -= dist(point1a, point1b);
           distance_other -= dist(point2a, point2b);
           intersection = point1a;
           return intersection;
         }
         if (point1a_LON == point2b_LON && point1a_LAT == point2b_LAT) // should never happen
         {
           distance_this -= dist(point1a, point1b);
           distance_other -= dist(point2b, point2b);
           intersection = point1a;
           return intersection;
         }
         if (point1b_LON == point2a_LON && point1b_LAT == point2a_LAT) // should never happen
         {
           distance_this -= dist(point1b, point1b);
           distance_other -= dist(point2a, point2b);
           intersection = point1b;
           return intersection;
         }
         if (point1b_LON == point2b_LON && point1b_LAT == point2b_LAT)
         {
           distance_this -= dist(point1b, point1b);
           distance_other -= dist(point2b, point2b);
           intersection = point1b;
           return intersection;
         }

          // check whether points 2a and 2b are on different sides of line [1a,1b]...
          double result1 = (point1b_LON - point1a_LON) * (point2a_LAT - point1b_LAT) - (point1b_LAT - point1a_LAT) * (point2a_LON - point1b_LON);
          double result2 = (point1b_LON - point1a_LON) * (point2b_LAT - point1b_LAT) - (point1b_LAT - point1a_LAT) * (point2b_LON - point1b_LON);
          bool result;
          result = (result1*result2 <= 0) ? true:false;

          // ... and check vice-versa (check whether points 1a and 1b are on different sides of line [2a,2b]
          if (result)
          {
            result1 = (point2b_LON - point2a_LON) * (point1a_LAT - point2b_LAT) - (point2b_LAT - point2a_LAT) * (point1a_LON - point2b_LON);
            result2 = (point2b_LON - point2a_LON) * (point1b_LAT - point2b_LAT) - (point2b_LAT - point2a_LAT) * (point1b_LON - point2b_LON);
            result = (result1*result2 <= 0) ? true:false;
          }

          if (result)
          {
            // calculate the exact intersection point
            //if (point1a_LAT != point2a_LAT || point1a_LON != point2a_LON || point1b_LAT != point2b_LAT || point1b_LON != point2b_LON)
            double intersection_LON = ((point1a_LON*point1b_LAT-point1a_LAT*point1b_LON)*(point2a_LON-point2b_LON)-(point1a_LON-point1b_LON)*(point2a_LON*point2b_LAT-point2a_LAT*point2b_LON))/((point1a_LON-point1b_LON)*(point2a_LAT-point2b_LAT)-(point1a_LAT-point1b_LAT)*(point2a_LON-point2b_LON));
            double intersection_LAT = ((point1a_LON*point1b_LAT-point1a_LAT*point1b_LON)*(point2a_LAT-point2b_LAT)-(point1a_LAT-point1b_LAT)*(point2a_LON*point2b_LAT-point2a_LAT*point2b_LON))/((point1a_LON-point1b_LON)*(point2a_LAT-point2b_LAT)-(point1a_LAT-point1b_LAT)*(point2a_LON-point2b_LON));
            intersection = boost::make_tuple(intersection_LAT, intersection_LON, int64_t(-1));
            distance_this -= dist(intersection, point1b);
            distance_other -= dist(intersection, point2b);
            return intersection;
          }
        }
      }
    }
    else
    {
      distance_this = 0;
      distance_other = 0;
      return this->node_at(CENTER,0);
    }
  }
  distance_this = -1;
  distance_other = -1;
  return point_with_id_t();
}

double Lanelet::curveRadius(const point_with_id_t &pt1, const point_with_id_t &pt2, const point_with_id_t &pt3) const
{
  // transform points to relative metric positions, pt1 being at (0,0)
  boost::tuple<double, double> pt2_metric = vec(pt1, pt2);
  boost::tuple<double, double> pt3_metric = vec(pt1, pt3);
  boost::tuple<double, double> pt1_metric = vec(pt1, pt1);

  double const x1 = boost::get<X>(pt1_metric), y1 = boost::get<Y>(pt1_metric);
  double const x2 = boost::get<X>(pt2_metric), y2 = boost::get<Y>(pt2_metric);
  double const x3 = boost::get<X>(pt3_metric), y3 = boost::get<Y>(pt3_metric);
  //calculation based on http://www.arndt-bruenner.de/mathe/scripts/kreis3p.htm
  double const nominator = -(pow(x1,2.) + pow(y1,2.))/(x1-x3) - (pow(x2,2.) + pow(y2,2.))/(x1-x2) + (pow(x1,2.) + pow(y1,2.))/(x1-x2) + (pow(x3,2.) + pow(y3,2.))/(x1-x3);
  double const denominator = ( (y1-y3)/(x1-x3) - (y1-y2)/(x1-x2) );
  double c = -nominator / denominator;
  double const b = -((y1-y3)/(x1-x3) * c - (pow(x1,2.) + pow(y1,2.))/(x1-x3) + (pow(x3,2.) + pow(y3,2.))/(x1-x3));
  double const a = x1*b + y1*c - (pow(x1,2.) + pow(y1,2.));
  double const radius = sqrt(pow(b/2.0,2.) + pow(c/2.0,2.) - a);
  return std::isnan(radius) ? 50000.0 : radius;
}

double Lanelet::maxCurveSpeed(const point_with_id_t &source, double max_lateral_acceleration)
{
  std::size_t index_a, index_b, index_c, index_d;
  point_with_id_t source_projected = project(source, NULL, NULL, &index_b, &index_c);

  if (index_b != 0 && index_c != nodes(CENTER).size() - 1)// there are at least two previous and subsequent nodes at source, use interpolation
  {
    index_a = index_b - 1;
    index_d = index_c + 1;
    double distance_to_b = dist(node_at(CENTER,index_b), source_projected);
    double distance_to_c = dist(node_at(CENTER,index_c), source_projected);

    double previous_curve_radius = curveRadius(node_at(CENTER,index_a), node_at(CENTER,index_b), node_at(CENTER,index_c));
    double previous_max_curve_speed = sqrt(max_lateral_acceleration*previous_curve_radius); //in m/s
    double subsequent_curve_radius = curveRadius(node_at(CENTER,index_b), node_at(CENTER,index_c), node_at(CENTER,index_d));
    double subsequent_max_curve_speed = sqrt(max_lateral_acceleration*subsequent_curve_radius); //in m/s

    //double curve_radius = (distance_to_b * previous_curve_radius + distance_to_c * subsequent_curve_radius) / (distance_to_b + distance_to_c);
    //double max_curve_speed2 = sqrt(max_lateral_acceleration*curve_radius); //in m/s
    double max_curve_speed = (distance_to_b * previous_max_curve_speed + distance_to_c * subsequent_max_curve_speed) / (distance_to_b + distance_to_c);
    return max_curve_speed;
  }
  else if (index_b == 0 && index_c != nodes(CENTER).size() - 1)// there is only one previous node and at least two subsequent nodes at source, use subsequent_curve_radius
  {
    index_d = index_c + 1;
    double subsequent_curve_radius = curveRadius(node_at(CENTER,index_b), node_at(CENTER,index_c), node_at(CENTER,index_d));
    double subsequent_max_curve_speed = sqrt(max_lateral_acceleration*subsequent_curve_radius); //in m/s
    return subsequent_max_curve_speed;
  }
  else if (index_b != 0 && index_c == nodes(CENTER).size() - 1)// there is only one subsequent node and at least two previous nodes at source, use subsequent_curve_radius
  {
    index_a = index_b - 1;
    double previous_curve_radius = curveRadius(node_at(CENTER,index_a), node_at(CENTER,index_b), node_at(CENTER,index_c));
    double previous_max_curve_speed = sqrt(max_lateral_acceleration*previous_curve_radius); //in m/s
    return previous_max_curve_speed;
  }
  else // lanelet consists of only two nodes, curve_radius is infinite
  {
    double curve_radius = 50000; // as curve_radius()'s upper bound is also 50000
    double max_curve_speed = sqrt(max_lateral_acceleration*curve_radius); //in m/s
    return max_curve_speed;
  }
}


std::ostream &operator<<(std::ostream &out, const lanelet_ptr_t &lanelet)
{
    boost::format fmt("<Lanelet %i>");
    out << (fmt % lanelet->id());
    return out;
}


double LLet::Lanelet::distance_from_center_line_to(const LLet::point_with_id_t& point) const
{
  return (boost::get<CENTER>(bounds()))->absolute_distance(point);
}
