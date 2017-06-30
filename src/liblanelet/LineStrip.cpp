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

#include "LineStrip.hpp"
#include <deque>
#include <algorithm>
#include <stdexcept>
#include <cassert>

#include <boost/format.hpp>
#include <boost/foreach.hpp>

using namespace LLet;

BoundingBox LineStrip::bb() const
{
    const std::vector< point_with_id_t >& _pts = this->pts();
    BoundingBox b(_pts.front());

    BOOST_FOREACH( const point_with_id_t& p, _pts )
    {
        b.extend_point(p);
    }

    return b;
}

ReversedLineStrip::ReversedLineStrip(boost::shared_ptr< LineStrip > parent) : _parent(parent)
{
    const std::vector< point_with_id_t >& ins = _parent->pts();
    _pts.insert( _pts.end(), ins.rbegin(), ins.rend() );
}

OSMLineStrip::OSMLineStrip()
{
}

const std::vector<point_with_id_t>& ReversedLineStrip::pts() const
{
  return this->_pts;
}

std::vector<point_with_id_t>& ReversedLineStrip::pts()
{
  return this->_pts;
}

const std::vector<point_with_id_t>& OSMLineStrip::pts() const
{
  return this->_pts;
}

std::vector<point_with_id_t>& OSMLineStrip::pts()
{
  return this->_pts;
}

double LineStrip::length(std::size_t start_index, std::size_t end_index) const
{
  std::size_t end = std::min(end_index, pts().size()-1);

  assert((start_index <= end) && "start_index must be <= min(end_index, pts().size()-1)");

  double result = 0.;
  for (std::size_t i=start_index; i<end; ++i)
  {
    result += abs(vec(pts()[i], pts()[i+1]));
  }
  return result;
}

double LineStrip::length() const
{
  return length(0, pts().size()-1);
}

std::vector<double> LineStrip::length_ratios() const
{
  std::vector<double> ratios;
  const double length = this->length();
  if (length == 0.) // no length -> no ratios
  {
    return ratios;
  }

  for (std::size_t i=0; i<pts().size()-1; ++i)
  {
    ratios.push_back(dist(pts()[i], pts()[i+1]) / length);
  }
  return ratios;
}

std::vector<double> LineStrip::accumulated_length_ratios() const
{
  std::vector<double> accumulated_ratios;
  const std::vector<double> ratios = length_ratios();

  double accumulated_ratio = 0.;
  for (std::size_t i=0; i<ratios.size(); ++i)
  {
    accumulated_ratio += ratios[i];
    accumulated_ratios.push_back(accumulated_ratio);
    assert(accumulated_ratio >= 0.0);
    assert(accumulated_ratio <= 1.0 + 1e-9);
  }
  return accumulated_ratios;
}

double LineStrip::signed_distance(const point_with_id_t& point) const
{
  assert((pts().empty() == false) && "Cannot calculate distance when line stip has no points");
  double smallest_distance = std::numeric_limits<double>::infinity();
  std::vector<point_xy_t> metric_points;

  for (std::size_t i=0; i< pts().size(); ++i)
  {
    metric_points.push_back(vec(pts()[0], pts()[i]));
  }
  // convert query point to metric local CS:
  point_xy_t query = vec(pts()[0], point);

  bool start_limited = true;
  bool end_limited = true;
  // calculate the distance to each line, store the (absolute) smallest
  for (std::size_t i=0; i<metric_points.size()-1; ++i)
  {
    if (i==0)
    {
      start_limited = false;
    }
    if (i+2==metric_points.size())
    {
      end_limited = false;
    }
    const double distance = signed_distance_to_line_segment(query, boost::make_tuple(metric_points[i], metric_points[i+1]),
                                                            start_limited, end_limited);
    // std::cout << "Distance for point " << i << " is " << distance << std::endl;
    if (std::abs(distance) < std::abs(smallest_distance))
    {
      // std::cout << "abs(distance) < abs(smallest_distance) : " << std::abs(distance) << " < " << std::abs(smallest_distance) << std::endl;
      smallest_distance = distance;
    }
  }
  // std::cout << "Smallest distance = " << smallest_distance << std::endl;
  return smallest_distance;
}

std::vector<point_xy_t> LineStrip::as_metric(point_with_id_t gnss_reference_point) const
{
  std::vector<point_xy_t> result;
  for (std::size_t i=0; i<pts().size(); ++i)
  {
    result.push_back(vec(gnss_reference_point, pts()[i]));
  }
  return result;
}

void LineStrip::from_metric(const point_with_id_t& gnss_reference_point, const std::vector<point_xy_t>& metric)
{
  pts().clear();
  for (std::size_t i=0; i<metric.size(); ++i)
  {
    pts().push_back(from_vec(gnss_reference_point, metric[i]));
  }
}

void LineStrip::interpolate_spline(double interpolation_distance)
{
  if (pts().size() < 4)
  {
    // todo: instead of aborting here use the same points multiple times
    std::cerr << "Strip cannot be spline interpolated, this would require at least 4 points!" << std::endl;
    return;
  }

  const point_with_id_t reference = this->pts()[0];
  const std::vector<point_xy_t> metric = this->as_metric(reference);
  std::vector<point_xy_t> metric_interp;

  // The first point is always valid
  metric_interp.push_back(metric[0]);

  // the first segment: (index 0 <-> index 1)
  const double distance_seg1 = dist(metric[0], metric[1]);

  if (distance_seg1 > interpolation_distance) // check if interpolation is necessary
  {
    double remaining_distance = distance_seg1;
    std::size_t counter = 1;
    point_xy_t interpol;
    double interpolation_ratio = 0.;

    while (remaining_distance > interpolation_distance)
    {
      interpolation_ratio = counter * interpolation_distance / distance_seg1;

      interpol = LLet::interpolate_spline(metric[0], // p-1, here = p0
                                          metric[0], // p0
                                          metric[1], // p+1
                                          metric[2], // p+2
                                          interpolation_ratio);
      metric_interp.push_back(interpol);
      remaining_distance -= interpolation_distance;
      ++counter;
    }
  }

  // We do not remove any points, so we add the next point no matter how far it is
  metric_interp.push_back(metric[1]);

  // all other segments except the last two: 1 < index < n-3
  for (std::size_t i=1; i<metric.size()-3; ++i)
  {
    const double distance = dist(metric[i], metric[i+1]);
    if (distance > interpolation_distance) // check if interpolation is necessary
    {
      double remaining_distance = distance;
      std::size_t counter = 1;
      point_xy_t interpol;
      double interpolation_ratio = 0.;

      while (remaining_distance > interpolation_distance)
      {
        interpolation_ratio = counter * interpolation_distance / distance;

        interpol = LLet::interpolate_spline(metric[i-1],
                                            metric[i],
                                            metric[i+1],
                                            metric[i+2],
                                            interpolation_ratio);
        metric_interp.push_back(interpol);
        remaining_distance -= interpolation_distance;
        ++counter;
      }
    }

    // We do not remove any points, so we add the next point no matter how far it is
    metric_interp.push_back(metric[i+1]);
  }

  // the last two segments:  (index n-3 <-> n-2) and (index n-2 <-> n-1)
  const std::size_t n = metric.size();
  const double distance_seg_n_1 = dist(metric[n-3], metric[n-2]);
  if (distance_seg_n_1 > interpolation_distance) // check if interpolation is necessary
  {
    double remaining_distance = distance_seg_n_1;
    std::size_t counter = 1;
    double interpolation_ratio = 0.;
    point_xy_t interpol;

    while (remaining_distance > interpolation_distance)
    {
      interpolation_ratio = counter * interpolation_distance / distance_seg_n_1;

      interpol = LLet::interpolate_spline(metric[n-4], // p-1
                                          metric[n-3], // p0
                                          metric[n-2], // p+1
                                          metric[n-1], // p+2
                                          interpolation_ratio);
      metric_interp.push_back(interpol);
      remaining_distance -= interpolation_distance;
      ++counter;
    }
  }

  // We do not remove any points, so we add the next point no matter how far it is
  metric_interp.push_back(metric[n-2]);

  const double distance_seg_n = dist(metric[n-2], metric[n-1]);
  if (distance_seg_n > interpolation_distance)  // check if interpolation is necessary
  {
    double remaining_distance = distance_seg_n;
    std::size_t counter = 1;
    double interpolation_ratio = 0;
    point_xy_t interpol;

    while (remaining_distance > interpolation_distance)
    {
      interpolation_ratio = counter * interpolation_distance / distance_seg_n;

      interpol = LLet::interpolate_spline(metric[n-3], // p-1
                                          metric[n-2], // p0
                                          metric[n-1], // p+1
                                          metric[n-1], // p+2, here p+2 = p+1
                                          interpolation_ratio);
      metric_interp.push_back(interpol);
      remaining_distance -= interpolation_distance;
      ++counter;
    }
  }

  // We do not remove any points, so we add the next point no matter how far it is
  metric_interp.push_back(metric[n-1]);

  from_metric(reference, metric_interp);
}


double LineStrip::signed_distance_to_line_segment(const point_xy_t& point, const boost::tuple<point_xy_t, point_xy_t>& line_segment,
                                                  bool line_origin_limited, bool line_end_limited) const
{
  using boost::get;

  const point_xy_t direction = boost::make_tuple(get<X>(get<1>(line_segment)) - get<X>(get<0>(line_segment)),
                                              get<Y>(get<1>(line_segment)) - get<Y>(get<0>(line_segment)));

  const point_xy_t origin2point = boost::make_tuple(get<X>(point) - get<X>(get<0>(line_segment)),
                                                 get<Y>(point) - get<Y>(get<0>(line_segment)));

  const point_xy_t end2point = boost::make_tuple(get<X>(point) - get<X>(get<1>(line_segment)),
                                              get<Y>(point) - get<Y>(get<1>(line_segment)));

  point_xy_t dir = direction;
  normalize(dir);

  const double line_dist = get<X>(dir) * get<Y>(origin2point) - get<Y>(dir) * get<X>(origin2point);

  if (!line_origin_limited && !line_end_limited) // open in both directions
  {
    return line_dist;
  }

  const double line_offset = dot(origin2point, dir);

  if (line_origin_limited && line_offset < 0.)
  {
    return (line_dist>0. ? 1. : -1.) * abs(origin2point);
  }
  else if (line_end_limited && line_offset > abs(direction))
  {
    return (line_dist>0. ? 1. : -1.) * abs(end2point);
  }
  else
  {
    return line_dist;
  }
}

double LineStrip::absolute_distance_to_line_segment(const point_xy_t& point, const boost::tuple<point_xy_t, point_xy_t>& line_segment) const
{

  using boost::get;

  const point_xy_t direction = boost::make_tuple(get<X>(get<1>(line_segment)) - get<X>(get<0>(line_segment)),
                                              get<Y>(get<1>(line_segment)) - get<Y>(get<0>(line_segment)));

  const point_xy_t origin2point = boost::make_tuple(get<X>(point) - get<X>(get<0>(line_segment)),
                                                 get<Y>(point) - get<Y>(get<0>(line_segment)));

  const point_xy_t end2point = boost::make_tuple(get<X>(point) - get<X>(get<1>(line_segment)),
                                              get<Y>(point) - get<Y>(get<1>(line_segment)));

  point_xy_t dir = direction;
  normalize(dir);

  const double line_offset = dot(origin2point, dir);

  if (line_offset < 0.)
  {
    return abs(origin2point);
  }
  if (line_offset > abs(direction))
  {
    return abs(end2point);
  }
  return std::abs(get<X>(dir) * get<Y>(origin2point) - get<Y>(dir) * get<X>(origin2point));
}

const std::vector<point_with_id_t>& CompoundLineStrip::pts() const
{
  return this->_pts;
}

std::vector<point_with_id_t>& CompoundLineStrip::pts()
{
  return this->_pts;
}

bool CompoundLineStrip::insertLineStrip(const boost::shared_ptr<LineStrip> &s)
{
  const std::vector< point_with_id_t >& ins = s->pts();
  if( boost::get<ID>(this->_pts.back()) == boost::get<ID>(s->pts().front()))
  {
    this->_pts.insert(this->_pts.end(), ins.begin(), ins.end());
    return true;
  }
  else if( boost::get<ID>(this->_pts.front()) == boost::get<ID>(s->pts().back()))
  {
    this->_pts.insert(this->_pts.begin(), ins.begin(), ins.end());
    return true;
  }
  else
    return false;
}

CompoundLineStrip::CompoundLineStrip(const std::vector<boost::shared_ptr<LineStrip> > &strips)
{
  assert( strips.size() );

  std::deque< boost::shared_ptr<LineStrip> > open(strips.begin(), strips.end());

  const std::vector< point_with_id_t >& ins = open.front()->pts();

  _pts.insert(_pts.end(), ins.begin(), ins.end());
  open.pop_front();



    while(!open.empty())
    {
        boost::shared_ptr<LineStrip> const strip = open.front();
        open.pop_front();

        if( !insertLineStrip(strip) )
        {
            strip_ptr_t const reversed = strip_ptr_t(new ReversedLineStrip(strip));
            if (!insertLineStrip(reversed))
            {
                if (open.empty())
                {
                    throw std::runtime_error("there seems to be an error in a line strip group: cannot concatenate all.");
                }
                open.push_back(strip);
            }
        }
    }

    std::vector< point_with_id_t >::iterator new_end = std::unique(_pts.begin(), _pts.end());
    _pts.erase(new_end, _pts.end());
}

point_with_id_t LineStrip::point_at_distance(double distance, double *angle) const
{
    double const len = length();
    if (distance <= -len)
    {
        return pts().front();
    }
    else if (distance >= len)
    {
        return pts().back();
    }

    bool const from_back = distance < 0.0;
    while (distance<0)
    {
        distance += len;
    }

    std::vector<point_with_id_t> points = pts();
    point_with_id_t const & base = points.front();
    LocalGeographicCS cs(boost::get<LLet::LAT>(base), boost::get<LLet::LON>(base));
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        point_xy_t const a = vec(base, points[i-1]);
        point_xy_t const b = vec(base, points[i]);
        double const currentDistance = dist(a,b);
        if (distance > currentDistance)
        {
            distance -= currentDistance;
        }
        else
        {
            double const t = distance / currentDistance;
            const double& ax = boost::get<LLet::X>(a);
            const double& ay = boost::get<LLet::Y>(a);
            const double& bx = boost::get<LLet::X>(b);
            const double& by = boost::get<LLet::Y>(b);
            point_xy_t const result = boost::make_tuple(ax+t*(bx-ax), ay+t*(by-ay));
            if (angle)
            {
                *angle = atan2(by-ay, bx-ax);
            }
            return from_vec(base, result);
        }
    }

    return from_back ? points.front() : points.back(); // abs(distance) > length
}


double LLet::LineStrip::absolute_distance(const LLet::point_with_id_t& point) const
{
  assert((pts().size() > 1) && "Cannot calculate distance when line stip has too few points");
  std::vector<point_xy_t> metric_points;

  for (std::size_t i=0; i< pts().size(); ++i)
  {
    metric_points.push_back(vec(pts()[0], pts()[i]));
  }
  // convert query point to metric local CS:
  point_xy_t query = vec(pts()[0], point);

  // calculate the absolute distance to each line and find the smallest
  double smallest_distance = absolute_distance_to_line_segment(query, boost::make_tuple(metric_points[0], metric_points[1]));
  for (std::size_t i=1; i<metric_points.size()-1; ++i)
  {
    const double distance = absolute_distance_to_line_segment(query, boost::make_tuple(metric_points[i], metric_points[i+1]));
    smallest_distance = std::min(smallest_distance, distance);
  }
  return smallest_distance;
}
