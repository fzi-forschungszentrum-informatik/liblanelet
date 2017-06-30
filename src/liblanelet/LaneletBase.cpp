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

#include "LaneletBase.hpp"

#include <algorithm>
#include <boost/foreach.hpp>

using namespace LLet;

LaneletBase::LaneletBase()
{
}

const point_with_id_t& LaneletBase::node_at(SIDE bound, int32_t n) const
{
    size_t L = nodes(bound).size();

    int32_t index = n >= 0 ? n : L + n;

    if(index >= int(L) || index < 0)
    {
        throw std::runtime_error("node_at(): index out of range.");
    }

    else
    {
        return nodes(bound)[index];
    }
}

const std::vector< point_with_id_t >& LaneletBase::nodes(SIDE bound) const
{
  switch(bound)
  {
    case LEFT:   return boost::get<LEFT>(bounds())->pts();
    case RIGHT:  return boost::get<RIGHT>(bounds())->pts();
    case CENTER: return boost::get<CENTER>(bounds())->pts();
    default: assert(false); return boost::get<RIGHT>(bounds())->pts();
  }
}

BoundingBox LaneletBase::bb() const
{
    strip_ptr_t left, right, center;
    boost::tie(left, right, center) = bounds();
    BoundingBox _bb = left->bb();
    _bb.extend_box( right->bb() );
    return _bb;
}

bool LaneletBase::fits_before(const lanelet_base_ptr_t &other) const
{
    if( this->node_at(LEFT, -1) != other->node_at(LEFT, 0) )
        return false;
    if( this->node_at(RIGHT, -1) != other->node_at(RIGHT, 0) )
        return false;
    return true;
}

bool LaneletBase::fits_beside(const lanelet_base_ptr_t &other, SIDE side) const
{
  SIDE const other_side = side == LEFT ? RIGHT: LEFT;
  std::string const anticross = side == LEFT ? "right": "left";
  bool const can_cross = this->node_at(other_side, 0) == other->node_at(side, 0) &&
                         this->node_at(other_side, -1) == other->node_at(side, -1);
  if (can_cross)
  {
    if (hasAttribute("nocross"))
    {
      std::string const nocross = attribute("nocross");
      if (nocross == anticross || nocross == "both")
      {
        return false;
      }
    }

    return true;
  }

  return false;
}

double LaneletBase::length(LLet::SIDE side) const
{
    double res = 0;
    const std::vector< point_with_id_t >& _nodes =  nodes(side);
    for (std::size_t i = 1; i < _nodes.size(); ++i)
    {
        res += dist(_nodes[i-1], _nodes[i]);
    }

    return res;
}
