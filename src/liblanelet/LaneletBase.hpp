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

#include <vector>

#include "lanelet_point.hpp"
#include "LineStrip.hpp"
#include "BoundingBox.hpp"
#include "RegulatoryElement.hpp"
#include "ImportExport.h"

namespace LLet
{

enum SIDE
{
    LEFT = 0,
    RIGHT = 1,
    CENTER = 2
};

class LaneletBase;

typedef boost::tuple< double, double > coord_t;
typedef boost::shared_ptr< LaneletBase > lanelet_base_ptr_t;

class LANELET_IMPORT_EXPORT LaneletBase : public HasAttributes
{
public:

    typedef boost::tuple<strip_ptr_t, strip_ptr_t, strip_ptr_t> bounds_container_t;

    LaneletBase();

    /// returns the (lat, lon) pair at the specified index. Throws if index is out of range. If index is negative, it will return
    /// the -nth element from the back. n=0 refers to the first, n=-1 refers to the last element.
    virtual const point_with_id_t& node_at( SIDE bound, int64_t n ) const;

    /// returns the vector of points describing the left or right bound.
    virtual const std::vector< point_with_id_t >& nodes( SIDE bound ) const;

    /// returns the left and right line strip.
    virtual const bounds_container_t& bounds() const = 0;

    virtual BoundingBox bb() const;
    virtual const std::vector< regulatory_element_ptr_t >& regulatory_elements() const = 0;

    bool fits_before(const lanelet_base_ptr_t& other) const;
    bool fits_beside(const lanelet_base_ptr_t& other, SIDE side) const;
    double length(LLet::SIDE side = LLet::RIGHT) const;
};

}
