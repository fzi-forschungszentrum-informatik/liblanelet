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
 
#pragma once
#include <boost/variant.hpp>
#include <vector>
#include <string>
#include <memory>

#include "LaneletFwd.hpp"
#include "lanelet_point.hpp"
#include "LineStrip.hpp"
#include "Attribute.hpp"

namespace LLet
{

typedef boost::variant< lanelet_ptr_t, strip_ptr_t, point_with_id_t > member_variant_t;
typedef std::pair< std::string, member_variant_t > regulatory_element_member_t;

class RegulatoryElement;

typedef boost::shared_ptr< RegulatoryElement > regulatory_element_ptr_t;

class LANELET_IMPORT_EXPORT RegulatoryElement : public HasAttributes
{
public:
    RegulatoryElement( int32_t id );
    int32_t id() const;
    const std::vector< regulatory_element_member_t >& members() const;
    std::vector< member_variant_t > members(const std::string &role) const;

    std::vector< regulatory_element_member_t >& members();

private:
    std::vector< regulatory_element_member_t > _members;
    const int32_t _id;
    AttributeMap _attributes;
};

}
