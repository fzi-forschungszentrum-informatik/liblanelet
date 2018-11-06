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

#include <boost/math/special_functions.hpp>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <boost/variant/get.hpp>
#include "LocalGeographicCS.hpp"
#include "normalize_angle.hpp"
#include "Attribute.hpp"

namespace LLet
{


enum LATLON_COORDS
{
    LAT = 0,
    LON = 1,
    ID = 2
};

enum XY_COORDS
{
    X = 0,
    Y = 1
};

class point_with_id_t : public boost::tuple<double, double, int64_t>, public HasAttributes {
public:
  point_with_id_t(const boost::tuple<double, double, int64_t> &tuple = boost::tuple<double, double, int64_t>(0.0, 0.0, -1)) :
    boost::tuple<double, double, int64_t>(tuple)
  {
    // nothing to do
  }

#ifdef _WIN32 // The tuple implementation of MSVC10 fails on the auto-generated copy ctor
    point_with_id_t(const point_with_id_t &other) :
      HasAttributes(other)
    {
        boost::get<LLet::LAT>(*this) = boost::get<LLet::LAT>(other);
        boost::get<LLet::LON>(*this) = boost::get<LLet::LON>(other);
        boost::get<LLet::ID>(*this) = boost::get<LLet::ID>(other);
    }
#endif

};

inline bool operator==(const point_with_id_t& lhs, const point_with_id_t& rhs)
{
  return boost::get<0>(lhs) == boost::get<0>(rhs)
      && boost::get<1>(lhs) == boost::get<1>(rhs)
      && boost::get<2>(lhs) == boost::get<2>(rhs);
}
inline bool operator!=(const point_with_id_t& lhs, const point_with_id_t& rhs){return !(lhs == rhs);}

typedef boost::tuple< double, double > point_xy_t;
typedef std::vector<point_xy_t> vertex_container_t;

inline
boost::tuple< double, double > vec( const point_with_id_t& a, const point_with_id_t& b )
{
    using boost::get;

    LocalGeographicCS cs(get<LAT>(a), get<LON>(a));

    double ax, ay, bx, by;
    boost::tie(ax, ay) = cs.ll2xy(get<LAT>(a), get<LON>(a));
    boost::tie(bx, by) = cs.ll2xy(get<LAT>(b), get<LON>(b));

    double dx = bx -  ax;
    double dy = by -  ay;

    return boost::make_tuple(dx, dy);
}

//! Calculate the absolute point from absolute point \a a with relative offset \a vec and the given id \a id
inline point_with_id_t from_vec(const point_with_id_t& a, const point_xy_t& vec, const int64_t id = -1)
{
  using boost::get;

  LocalGeographicCS cs(get<LAT>(a), get<LON>(a));
  const boost::tuple<double, double>& ll = cs.xy2ll(get<X>(vec), get<Y>(vec));
  return boost::make_tuple(get<LAT>(ll), get<LON>(ll), id);
}

inline
double abs( const boost::tuple< double, double > &v )
{
    using boost::get;
    using boost::math::hypot;
    return hypot( get<X>(v), get<Y>(v) );
}

//! Calculate the distance between (metric) a and b
inline double dist(const point_xy_t& a, const point_xy_t& b)
{
  using boost::get;
  return abs(boost::make_tuple(get<X>(b) - get<X>(a), get<Y>(b) - get<Y>(a)));
}

//! Normalize the vector \a vec
inline void normalize(boost::tuple<double, double>& vec)
{
  using boost::get;
  const double length = abs(vec);
  assert(length != 0 && "The given vector's length is 0.");
  vec = boost::make_tuple(get<X>(vec) / length, get<Y>(vec) / length);
}

//! Calculate the distance between (gnss) a and b
inline double dist( const point_with_id_t& a, const point_with_id_t& b )
{
    return abs(vec(a, b));
}

inline
double scalar_product( const boost::tuple< double, double >& a, const boost::tuple< double, double >& b )
{
    using boost::get;
    return get<X>(a) * get<X>(b) + get<Y>(a) * get<Y>(b);
}

//! Project p on line
inline
point_xy_t projected(const point_xy_t& p, const vertex_container_t &line, double* angle=0, std::size_t* index=0, std::size_t* previous_index=0, std::size_t* subsequent_index=0)
{
  double min_distance = -1.0;
  double const px = boost::get<LLet::X>(p);
  double const py = boost::get<LLet::Y>(p);
  point_xy_t min_point = p;
  size_t const size = line.size();
  for (std::size_t i=1; i<size; ++i)
  {
    const double& ax = boost::get<LLet::X>(line[i-1]);
    const double& ay = boost::get<LLet::Y>(line[i-1]);
    const double& bx = boost::get<LLet::X>(line[i]);
    const double& by = boost::get<LLet::Y>(line[i]);

    point_xy_t const AP = point_xy_t(px-ax, py-ay);
    point_xy_t const AB = point_xy_t(bx-ax, by-ay);
    double const scalar_product_ab = scalar_product(AB, AB);
    double const lambda = scalar_product(AP, AB) / ( scalar_product_ab == 0.0 ? 1.0 : scalar_product_ab );

    double distance = 0.0;
    point_xy_t c;
    if (lambda <= 0.0)
    {
      c = line[i-1];
    }
    else if (lambda < 1.0)
    {
      c = point_xy_t(ax + lambda*(bx-ax), ay + lambda*(by-ay));
    }
    else
    {
      c = line[i];
    }
    distance = dist(p, c);
    if (min_distance < 0.0 || distance < min_distance)
    {
      min_distance = distance;
      min_point = c;
      if (angle)
      {
        *angle = atan2(boost::get<Y>(line[i])-boost::get<Y>(line[i-1]), boost::get<X>(line[i])-boost::get<X>(line[i-1]));
      }
      if (index)
      {
        *index = lambda < 0.5? i-1 : i;
      }
      if (subsequent_index)
      {
        *subsequent_index = i;
      }
      if (previous_index)
      {
        *previous_index = i-1;
      }
    }
  }

  return min_point;
}

inline
double angle( const boost::tuple< double, double >& a, const boost::tuple< double, double >& b )
{
    using boost::get;

    double sp = scalar_product(a, b);
    double cos_phi = sp / (abs(a) * abs(b));

    // sign for angle: test cross product
    double crossp_z = get<X>(a) * get<Y>(b) - get<Y>(a) * get<X>(b);
    double signum = boost::math::sign(crossp_z);
    double phi = normalize_angle(signum * std::acos(cos_phi));
    return phi;
}


template< typename T1, typename T2 >
inline
bool inrange(const T1& val, const T2& lo, const T2& hi)
{
    return val >= lo && val <= hi;
}

/*! Interpolate the given points using a Catmull-Rom polygon.
 *  Any \a ratio between 0 and 1 will interpolate in between \a p1 and \a p2.
 */
inline point_xy_t interpolate_spline(const point_xy_t& p0, const point_xy_t& p1,
                                     const point_xy_t& p2, const point_xy_t& p3, double ratio)
{
  /* Catmull-Rom coefficients:
   *
   * a0 = -0.5*p0 + 1.5*p1 - 1.5*p2 + 0.5*p3
   * a1 = p0 - 2.5*p1 + 2*p2 - 0.5*p3
   * a2 = -0.5*p0 + 0.5*p2
   */

  const point_xy_t a0 = boost::make_tuple(-0.5 * boost::get<X>(p0) + 1.5 * boost::get<X>(p1) - 1.5 * boost::get<X>(p2) + 0.5 * boost::get<X>(p3),
                                       -0.5 * boost::get<Y>(p0) + 1.5 * boost::get<Y>(p1) - 1.5 * boost::get<Y>(p2) + 0.5 * boost::get<Y>(p3));

  const point_xy_t a1 = boost::make_tuple(boost::get<X>(p0) - 2.5 * boost::get<X>(p1) + 2. * boost::get<X>(p2) - 0.5 * boost::get<X>(p3),
                                       boost::get<Y>(p0) - 2.5 * boost::get<Y>(p1) + 2. * boost::get<Y>(p2) - 0.5 * boost::get<Y>(p3));

  const point_xy_t a2 = boost::make_tuple(-0.5 * boost::get<X>(p0) + 0.5 * boost::get<X>(p2),
                                       -0.5 * boost::get<Y>(p0) + 0.5 * boost::get<Y>(p2));
  const double ratio_square = ratio * ratio;

  // Catmull-Rom polynom: result = a0 * ratio³ + a1 * ratio² + a2 * ratio + p1
  return boost::make_tuple(boost::get<X>(a0) * ratio * ratio_square + boost::get<X>(a1) * ratio_square + boost::get<X>(a2) * ratio + boost::get<X>(p1),
                        boost::get<Y>(a0) * ratio * ratio_square + boost::get<Y>(a1) * ratio_square + boost::get<Y>(a2) * ratio + boost::get<Y>(p1));
}

//! Calculate the dot product for vector \a and vector \b
inline double dot(const point_xy_t& a, const point_xy_t& b)
{
  return boost::get<LLet::X>(a) * boost::get<LLet::X>(b) +
      boost::get<LLet::Y>(a) * boost::get<LLet::Y>(b);
}

}
