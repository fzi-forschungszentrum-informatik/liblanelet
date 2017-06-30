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

#include "Attribute.hpp"
#include "lanelet_point.hpp"
#include "BoundingBox.hpp"
#include "ImportExport.h"

#include <vector>
#include <memory>
#include <boost/tuple/tuple.hpp>

namespace LLet
{

class LANELET_IMPORT_EXPORT LineStrip
{

public:

    // Read access
    virtual const std::vector< point_with_id_t >& pts() const = 0;

    // Write access. Be careful, this may invalidate lanelets!
    virtual std::vector< point_with_id_t >& pts() = 0;

    virtual BoundingBox bb() const;

    /*! Calculate the (metric) length when walking along the linestrip
     *  from \a start_index to \a end_index or the end of the line strip,
     *  whatever comes first.
     */
    double length(std::size_t start_index, std::size_t end_index) const;

    //! Calculate the (metric) length when walking along the whole linestrip
    double length() const;

    //! Calculate the length ratios for each line in the strip
    std::vector<double> length_ratios() const;

    //! Calculate the accumulated length ratios along the strip
    std::vector<double> accumulated_length_ratios() const;

    /*! Calculate the signed distance in meters from \a point to this strip.
     *  To do so, the distance to all of the strip's lines is calculated.
     *  The (absolute) shortest distance is then used to determine the relevant
     *  line and the signed distance to this closest line is returned.
     */
    double signed_distance(const point_with_id_t& point) const;

    /*! Calculate the absolute distance in meters from \a point to this strip.
     *  To do so the strip is taken as-is and not virtually extended at the ends
     *  of the strip. Instead the shortest absolute distance considering all of
     *  ths strip's points is used.
     */
    double absolute_distance(const point_with_id_t& point) const;

    //! Get a metric copy of the strip using the given \a gnss_reference_point for conversion
    std::vector<point_xy_t> as_metric(point_with_id_t gnss_reference_point) const;

    //! Create from metric points using the given \a gnss_reference_point for conversion
    void from_metric(const point_with_id_t& gnss_reference_point, const std::vector<point_xy_t>& metric);

    /*! Generate new points along the strip. Does not remove any.
     *  New points are sampled each time the \a interpolation_distance
     *  is greater then the euclidean distance between two point pairs.
     *  Interpolation methos is based on Catmull-Rom splines.
     */
    void interpolate_spline(double interpolation_distance);

    /*! Travel from the start of the strip (or from the end of the strip if distance is negative)
     * for a distance of abs(distance) and return the resulting point
     */
    point_with_id_t point_at_distance(double distance, double *angle=0) const;

private:

    /*! Calculate the signed distance in meters from a metric \a point to \a line segment.
     *  If \a line_origin_limited is \c false the segment will be considered as a half
     *  ray open at the semgent's origin. If \a line_end_limited is \c false the segment
     *  will be considered as a half ray open at the semgent's end.
     */
    double signed_distance_to_line_segment(const point_xy_t& point, const boost::tuple<point_xy_t, point_xy_t>& line_segment,
                                           bool line_origin_limited = true, bool line_end_limited = true) const;


    /*! Calculate the absolute distance in meters from a metric \a point to \a line segment.
     *  When walking over start or end the distance to the limiting point is used.
     */
    double absolute_distance_to_line_segment(const point_xy_t &point, const boost::tuple<point_xy_t, point_xy_t>& line_segment) const;
};

typedef boost::shared_ptr< LineStrip > strip_ptr_t;

class LANELET_IMPORT_EXPORT OSMLineStrip : public LineStrip
{
public:
    OSMLineStrip();
    AttributeMap _attributes;
    virtual const std::vector< point_with_id_t >& pts() const;
    virtual std::vector< point_with_id_t >& pts();

    std::vector< point_with_id_t > _pts;
private:
};

typedef boost::shared_ptr< OSMLineStrip > osm_strip_ptr_t;

class CompoundLineStrip : public LineStrip
{
public:
    CompoundLineStrip( const std::vector< boost::shared_ptr< LineStrip > > & strips );
    virtual const std::vector< point_with_id_t >& pts() const;
    virtual std::vector< point_with_id_t >& pts();

private:
    bool insertLineStrip(const boost::shared_ptr< LineStrip > &s);

    std::vector< point_with_id_t > _pts;

};

class LANELET_IMPORT_EXPORT ReversedLineStrip : public LineStrip
{
public:
    ReversedLineStrip( boost::shared_ptr< LineStrip > parent );

    virtual const std::vector< point_with_id_t >& pts() const;
    virtual std::vector< point_with_id_t >& pts();

private:
    const boost::shared_ptr< LineStrip > _parent;
    std::vector< point_with_id_t > _pts;


};

}
