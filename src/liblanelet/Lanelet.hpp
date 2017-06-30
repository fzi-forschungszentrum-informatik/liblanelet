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

#include "LaneletBase.hpp"
#include "Attribute.hpp"
#include "LineStrip.hpp"
#include "lanelet_point.hpp"
#include "RegulatoryElement.hpp"
#include "LaneletFwd.hpp"
#include "ImportExport.h"

#include <boost/tuple/tuple.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace LLet
{

class Polygon;

class LANELET_IMPORT_EXPORT Lanelet : public LaneletBase
{
public:
    Lanelet( );
    Lanelet( int32_t id, const strip_ptr_t& left, const strip_ptr_t& right );

    virtual const bounds_container_t& bounds() const;

    virtual const std::vector< regulatory_element_ptr_t >& regulatory_elements() const;
    std::vector< regulatory_element_ptr_t >& regulatory_elements();

    int32_t id() const;

    void add_regulatory_element( const regulatory_element_ptr_t& elem );

    /*! Determine if the lanelet, when interpreted as a polygon's area,
     *  is covering the given point's coordinates.
     * \param query The point to check
     * \return \c true, if so, \c else otherwise
     */
    bool covers_point(const point_with_id_t& query) const;

    /*!
     * Returns the distance (in meter) from the given point to the
     * (polygonal) bounds of the lanelet
     * \param point Point to calculate the distance to
     * \return 0.0 for points on the polygonal boundary of the lanelet,
     *  and a positive distance for points inside or outside the lanelet
     */
    double distance_to(const point_with_id_t &point) const;

    /*! Calculate the (absolute) distance (im meter) from the given point to the
     *  lanelet's center line.
     * \param point Point to calculate the distance to
     * \returns The absolute distance
     */
    double distance_from_center_line_to(const point_with_id_t& point) const;

    /*!
     * Matches the given position on the center polyline
     * \param source Source position to match on the closest point of the center polyline
     * \param angle When not null, the (driving) direction of the result position is stored here as
     * a rad value, range -pi..pi with 0 indicating east in counter clockwise direction
     * \param index When not null, the index next in line on the center polyline is stored here
     * \param previous_index When not null, the previous index on the center polyline is stored here
     * \param index When not null, the subsequent index on the center polyline is stored here
     * \return The source position projected on the closest point on the center polyline
     */
    point_with_id_t project(const point_with_id_t &source, double* angle=0, std::size_t* index=0, std::size_t* previous_index=0, std::size_t* subsequent_index=0) const;

    /*!
     * Calculates the distance along the lanelet from first node (node 0) to the projection of source
     * \param source the source point from which the projection is calculated, the projection is then used to
     * calculate the distance along the lane
     * \return the distance (1 dimensional) in meters
     */
    double distance_along_lanelet(const point_with_id_t &source) const;

    /*!
     * Calculates the point on the lanelet when following the lanelet from \a source for \a distance meters
     * \param source The source point from which the projection is calculated, the projection is then used to
     * calculate the result
     * \param result The resulting point on the lanelet, which is \a distance meters away (along lanelet) from \a source
     * if distance is too long for this lanelet, the resulting point is 0/0
     * \return the leftover distance (1 dimensional) until the end of the lanelet (can be negative)
     */
    double point_at_distance_along_lanelet(const point_with_id_t &source, point_with_id_t &result, double distance) const;

    /*!
     * Same as above, but starting at the beginning of the lanelet instead of at \a source
     */
    double point_at_distance_along_lanelet(point_with_id_t &result, double distance) const;

    /*! Calculate the (metric) bounding box of the lanelet.
     *  \param gnss_reference_point The reference point for conversion to metric coordinate system
     *  \param lower_left The lower values in x and y dimension of the bounding box rectangle
     *  \param upper_right The upper values in x and y dimension of the bounding box rectangle
     */
    void bounding_box(const point_with_id_t& gnss_reference_point, point_xy_t& lower_left, point_xy_t& upper_right) const;

    //! Read access to the lanelet interpreted as polygon
    boost::shared_ptr<const Polygon> polygon() const
    {
      return m_polygon;
    }

    /*!
     * Checks whether this lanelet intersects with other_lanelet (one single mutual point is enough to return true,
     * therefore following / merging / diverging / same lanes also result in true)
     * @note: only works if one of the other_lane's nodes lies within this lanelets polygon area (or other way round)!
     * \param other_lanelet the other lanelet to check the intersection with
     * \return whether there is an intersection or not
     */
    bool intersects(const lanelet_ptr_t other_lanelet);

    /*!
     * Calculates the first found intersection_point of this lanelet and other_lanelet (one single mutual point is enough,
     * therefore following / merging / diverging / same lanes also result in a intersection)
     * @note: normally there should only exist one intersection_point, but if there are multiple, only one is found!
     * \param other_lanelet the other lanelet to check the intersection with
     * \param distance_this reference which is set to the distance from start of this lanelet to crosspoint
     * \param distance_other reference which is set to the distance from start of other_lanelet to crosspoint
     * \return the intersection point (LAT,LON,-1), returns (0,0,-1) if no intersection is found
     */
    point_with_id_t intersection_point(const lanelet_ptr_t other_lanelet, double& distance_this, double& distance_other);


    /*!
     * Calculates the radius of the circle defined by the three points pt1, pt2, pt3
     * \param pt1, pt2, pt3 The three points which lie on the circle being calculated
     * \return the radius of the circle
     */
    double curveRadius(const point_with_id_t &pt1, const point_with_id_t &pt2, const point_with_id_t &pt3) const;

    /*!
     * Calculates the maximum senseful curve speed at the given position, considering maximal allowed lateral acceleration
     * \param source The position (which is being projected to this lanelet) to calculate the maximum senseful speed at
     * \param max_lateral_acceleration The maximum allowed lateral acceleration
     * \return the maximum senseful curve speed at the given position
     */
    double maxCurveSpeed(const point_with_id_t &source, double max_lateral_acceleration=2.);

private:

    /*! Calculates the center line strip out of the
     *  left and right boundary line strips and stores it within _bounds.
     *  The result may be accessed via the \c CENTER entry of \see bounds().
     */
    strip_ptr_t calculate_center_line_strip() const;

    int32_t _id;
    bounds_container_t _bounds;
    std::vector< regulatory_element_ptr_t > _regulatory_elements;
    boost::shared_ptr<Polygon> m_polygon;
};

}

LANELET_IMPORT_EXPORT std::ostream& operator<<( std::ostream& out, const LLet::lanelet_ptr_t& lanelet );
