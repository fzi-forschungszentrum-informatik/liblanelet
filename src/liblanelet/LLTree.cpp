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

#include "LLTree.hpp"

#include <algorithm>
#include <boost/foreach.hpp>

using namespace LLet;

void LLTree::insert(const lanelet_ptr_t& obj)
{
    int64_t index = _lanelets.size();
    _lanelets.push_back( obj );
    const BoundingBox bb = obj->bb();
    double min_data[2] = {bb.get<BoundingBox::SOUTH>(), bb.get<BoundingBox::WEST>()};
    double max_data[2] = {bb.get<BoundingBox::NORTH>(), bb.get<BoundingBox::EAST>()};
    _tree.Insert(min_data, max_data, index);
}

namespace
{

    bool callback(int64_t index, void* data )
    {
        std::vector< int64_t >* indices = static_cast< std::vector< int64_t >* >(data);
        indices->push_back(index);
        return true;
    }

}

std::vector< lanelet_ptr_t > LLTree::query( const BoundingBox& bb) const
{
    double min_data[2] = {bb.get<BoundingBox::SOUTH>(), bb.get<BoundingBox::WEST>()};
    double max_data[2] = {bb.get<BoundingBox::NORTH>(), bb.get<BoundingBox::EAST>()};

    std::vector< int64_t > indices;
    _tree.Search(min_data, max_data, callback, &indices);

    std::vector< lanelet_ptr_t > result(indices.size());

    // this replaces: std::transform(indices.begin(), indices.end(), result.begin(), [this](int64_t index){ return _lanelets[index];});
    size_t i = 0;
    BOOST_FOREACH(int64_t index, indices)
    {
      result[i++] = _lanelets[index];
    }

    return result;

}
