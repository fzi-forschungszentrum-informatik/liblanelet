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

#include "RegulatoryElement.hpp"

#include <boost/foreach.hpp>

using namespace LLet;

LLet::RegulatoryElement::RegulatoryElement(int64_t id) : _id(id)
{
}

int64_t RegulatoryElement::id() const
{
    return _id;
}

const std::vector< regulatory_element_member_t > &RegulatoryElement::members() const
{
    return _members;
}


std::vector< regulatory_element_member_t > &RegulatoryElement::members()
{
    return _members;
}

std::vector<member_variant_t> RegulatoryElement::members(const std::string &role) const
{
    std::vector< member_variant_t > result;
    BOOST_FOREACH( const regulatory_element_member_t& m, members() )
    {
        if(m.first == role)
        {
            result.push_back( m.second );
        }
    }
    return result;
}
