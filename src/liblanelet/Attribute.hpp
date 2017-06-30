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

#include "ImportExport.h"

#include <map>
#include <boost/lexical_cast.hpp>
#include <string>
#include <boost/format.hpp>

namespace LLet
{
struct AttributeValue
{
    AttributeValue( const std::string& v=std::string()) : value(v)
    {

    }

    std::string value;

    operator double() const
    {
        return as_double();
    }

    operator std::string() const
    {
        return value;
    }

    std::string as_string() const
    {
        return value;
    }

    double as_double() const
    {
        try
        {
            return boost::lexical_cast< double >(value);
        }
        catch(...)
        {
            // TODO: maybe catch more selectively ...
            boost::format fmt("AttributeValue '%s' not convertible to double.");
            throw std::runtime_error((fmt % value).str());
        }
    }

};

typedef std::map< std::string, AttributeValue > AttributeMap;

class LANELET_IMPORT_EXPORT HasAttributes
{
public:
    const AttributeValue& attribute(const std::string &key ) const;
    AttributeMap& attributes();
    const AttributeMap& attributes() const;
    bool hasAttribute(const std::string &key) const;
private:
    AttributeMap _attributes;
    static AttributeValue null_value;
};

}
