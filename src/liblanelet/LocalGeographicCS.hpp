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
 * \author  Julius Ziegler <ziegler@fzi.de>
 * \date    2012-03-06
 *
 */
//----------------------------------------------------------------------

#if !defined(LOCALGEOGRAPHICCS_HPP)
#define LOCALGEOGRAPHICCS_HPP

#include "convert_coordinates.hpp"

#include <boost/tuple/tuple.hpp>

#include <utility>

struct LocalGeographicCS
{
  LocalGeographicCS();
  LocalGeographicCS( double lat0, double lon0 );

  void set_origin( double lat0, double lon0 );

  void ll2xy( double lat, double lon, double& x, double& y ) const;
  void xy2ll( double x, double y, double& lat, double& lon ) const;

  boost::tuple<double, double> ll2xy( double lat, double lon ) const;
  boost::tuple<double, double> xy2ll( double x, double y ) const;

  // operate on containers
  template<class ItIn, class ItOut>
  void ll2xy( const ItIn& lat_begin, const ItIn& lat_end, const ItIn& lon_begin, const ItOut& x_begin, const ItOut& y_begin ) const;

  template<class ItIn, class ItOut>
  void xy2ll( const ItIn& x_begin, const ItIn& x_end, const ItIn& y_begin, const ItOut& lat_begin, const ItOut& lon_begin ) const;

private:
  double _scale;
  double _x0, _y0;
};

inline LocalGeographicCS::LocalGeographicCS( double lat0, double lon0 )
{
  set_origin( lat0, lon0 );
}

inline LocalGeographicCS::LocalGeographicCS()
{}

inline void LocalGeographicCS::set_origin( double lat0, double lon0 )
{
  _scale = convert_coordinates::lat_to_scale( lat0 );
  convert_coordinates::latlon_to_mercator( lat0, lon0, _scale, _x0, _y0 );
}

inline void LocalGeographicCS::ll2xy( double lat, double lon, double& x, double& y ) const
{
  convert_coordinates::latlon_to_mercator( lat, lon, _scale, x, y );
  x -= _x0;
  y -= _y0;
}

inline boost::tuple<double, double> LocalGeographicCS::ll2xy( double lat, double lon ) const
{
  double x, y;
  ll2xy( lat, lon, x, y );
  return boost::make_tuple( x, y );
}

inline void LocalGeographicCS::xy2ll( double x, double y, double& lat, double& lon ) const
{
  x += _x0;
  y += _y0;

  convert_coordinates::mercator_to_latlon( x, y, _scale, lat, lon );
}

inline boost::tuple<double, double> LocalGeographicCS::xy2ll( double x, double y ) const
{
  double lat, lon;
  xy2ll( x, y, lat, lon );
  return boost::make_tuple( lat, lon );
}

// operate on containers
template<class ItIn, class ItOut>
void LocalGeographicCS::ll2xy( const ItIn& lat_begin, const ItIn& lat_end, const ItIn& lon_begin, const ItOut& x_begin, const ItOut& y_begin ) const
{
  ItIn lat = lat_begin;
  ItIn lon = lon_begin;
  ItOut x = x_begin;
  ItOut y = y_begin;

  for( ; lat != lat_end; lat++, lon++, x++, y++ )
    {
      ll2xy( *lat, *lon, *x, *y );
    }
}

template<class ItIn, class ItOut>
void LocalGeographicCS::xy2ll( const ItIn& x_begin, const ItIn& x_end, const ItIn& y_begin, const ItOut& lat_begin, const ItOut& lon_begin ) const
{
  ItIn x = x_begin;
  ItIn y = y_begin;

  ItOut lat = lat_begin;
  ItOut lon = lon_begin;

  for( ; x != x_end; lat++, lon++, x++, y++ )
    xy2ll( *x, *y, *lat, *lon );
}

#endif
