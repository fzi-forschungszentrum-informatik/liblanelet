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

#include <boost/tuple/tuple.hpp>
#include <iostream>

#include "lanelet_point.hpp"

namespace LLet
{

struct BoundingBox
{
    enum COORDS
    {
        /// min lat
        SOUTH = 0,
        /// min lon
        WEST = 1,
        /// max lat
        NORTH = 2,
        /// max lon
        EAST = 3
    };

    boost::tuple< double, double, double, double > bb;

    template< int index >
    const double& get() const
    {
        return boost::get<index>(bb);
    }

    BoundingBox( const point_with_id_t& pt ) : bb(boost::make_tuple( boost::get<LAT>(pt), boost::get<LON>(pt), boost::get<LAT>(pt), boost::get<LON>(pt) ) )
    {

    }

    BoundingBox (const boost::tuple< double, double, double, double > &_bb) : bb(_bb)
    {

    }

    void extend_point( const point_with_id_t& pt )
    {
        boost::get<NORTH>(bb) = std::max( boost::get<NORTH>(bb), boost::get<LAT>(pt) );
        boost::get<SOUTH>(bb) = std::min( boost::get<SOUTH>(bb), boost::get<LAT>(pt) );
        boost::get<EAST>(bb) = std::max( boost::get<EAST>(bb), boost::get<LON>(pt) );
        boost::get<WEST>(bb) = std::min( boost::get<WEST>(bb), boost::get<LON>(pt) );
    }

    void extend_box( const BoundingBox& other )
    {
        boost::get<NORTH>(bb) = std::max( boost::get<NORTH>(bb), boost::get<NORTH>(other.bb) );
        boost::get<SOUTH>(bb) = std::min( boost::get<SOUTH>(bb), boost::get<SOUTH>(other.bb) );
        boost::get<EAST>(bb) = std::max( boost::get<EAST>(bb), boost::get<EAST>(other.bb) );
        boost::get<WEST>(bb) = std::min( boost::get<WEST>(bb), boost::get<WEST>(other.bb) );
    }
};
}

inline
std::ostream& operator<<(std::ostream& out, const LLet::BoundingBox& box )
{
    out << "foo";
    return out;
}


