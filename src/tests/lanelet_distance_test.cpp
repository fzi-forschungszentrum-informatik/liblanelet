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
 * \author  Sebastian Klemm <klemm@fzi.de>
 *
 */
//----------------------------------------------------------------------

#include <string>
#include <fstream>
#include <liblanelet/Polygon.hpp>
#include <liblanelet/Lanelet.hpp>
#include <liblanelet/LaneletMap.hpp>


using namespace LLet;

inline double sq(double x)
{
  return x * x;
}

void plotLaneletDistances(lanelet_ptr_t lanelet, const std::string& filename_without_suffix)
{
  point_with_id_t gnss_reference = lanelet->polygon()->gnssReferencePoint();
  const double grid_cell_size = 0.5;
  const double margin_percentual = 0.25;
  point_xy_t lower_left, upper_right;
  lanelet->polygon()->boundingBox(lower_left, upper_right);

  const double diag_length = abs(boost::make_tuple(boost::get<X>(upper_right) - boost::get<X>(lower_left),
                                                boost::get<Y>(upper_right) - boost::get<Y>(lower_left)));

  boost::get<X>(lower_left) = boost::get<X>(lower_left) - margin_percentual * diag_length;
  boost::get<Y>(lower_left) = boost::get<Y>(lower_left) - margin_percentual * diag_length;
  boost::get<X>(upper_right) = boost::get<X>(upper_right) + margin_percentual * diag_length;
  boost::get<Y>(upper_right) = boost::get<Y>(upper_right) + margin_percentual * diag_length;

  // calculate number of horizontal and vertical samples
  const double diff_x = boost::get<X>(upper_right) - boost::get<X>(lower_left);
  const double diff_y = boost::get<Y>(upper_right) - boost::get<Y>(lower_left);

  const std::size_t size_x = std::floor(diff_x / grid_cell_size);
  const std::size_t size_y = std::floor(diff_y / grid_cell_size);

  strip_ptr_t left  = boost::get<LEFT>(lanelet->bounds());
  strip_ptr_t right = boost::get<RIGHT>(lanelet->bounds());


  const std::string fn = filename_without_suffix + "_distance.gpldata";
  const std::string fn2 = filename_without_suffix + "_lanelet.gpldata";

  std::ofstream distance_file(fn.c_str(), std::ios::out);
  std::ofstream lanelet_file(fn2.c_str(), std::ios::out);

  // --- plot lanelet metric coordinates ---
  lanelet_file << "# Lanelet "<< lanelet->id() << " metric point data using reference point (lat, lon): "
               << boost::get<LAT>(gnss_reference) << ", " << boost::get<LON>(gnss_reference) << std::endl;
  for (std::size_t i=0; i<left->pts().size(); ++i)
  {
    point_xy_t metric = vec(gnss_reference, left->pts()[i]);
    lanelet_file << boost::get<X>(metric) << "  " << boost::get<Y>(metric) << std::endl;
  }
  // add empty lines to separate linestrips
  lanelet_file << std::endl << std::endl << std::endl;

  for (std::size_t i=0; i<right->pts().size(); ++i)
  {
    point_xy_t metric = vec(gnss_reference, right->pts()[i]);
    lanelet_file << boost::get<X>(metric) << "  " << boost::get<Y>(metric) << std::endl;
  }

  lanelet_file << std::endl;
  lanelet_file.close();

  // --- plot distance data ---
  distance_file << "# Lanelet "<< lanelet->id() << " distance data" << std::endl;

  // sample the bounding box and calculate the signed distance
  for (std::size_t iy=0; iy<size_y; ++iy)
  {
    for (std::size_t ix=0; ix<size_x; ++ix)
    {
      point_xy_t sample;
      boost::get<X>(sample) = boost::get<X>(lower_left) + ix * grid_cell_size;
      boost::get<Y>(sample) = boost::get<Y>(lower_left) + iy * grid_cell_size;
      const point_with_id_t sample_gnss = from_vec(gnss_reference, sample);

      const double d_left  = left->signed_distance(sample_gnss);
      const double d_right = right->signed_distance(sample_gnss);

      distance_file << boost::get<X>(sample) << "  " << boost::get<Y>(sample) << "  " << sq(0.5 * (d_left + d_right)) << std::endl;
    }
  }

  distance_file << std::endl;

  distance_file.close();

  const std::string fn_gnuplot = filename_without_suffix + ".gpl";
  std::ofstream gnuplot_file(fn_gnuplot.c_str(), std::ios::out);

  gnuplot_file << "# Gnuplot file. Draw with gnuplot -p "<< fn_gnuplot << std::endl;
  gnuplot_file << "set mapping cartesian" << std::endl;
  gnuplot_file << "set mouse" << std::endl;
  gnuplot_file << "splot '"<< fn << "' using 1:2:3:3 w p palette , '"<< fn2 <<"' using 1:2:(0) with lines" << std::endl;
  gnuplot_file.close();

  std::cout << "GnuPlot file written to " << fn_gnuplot << std::endl;
}


void plotMapDistances(const point_with_id_t& gnss_reference, lanelet_map_ptr_t map, const std::string& filename_without_suffix)
{
  const double grid_cell_size = 0.5;
  const double margin_percentual = 0.25;
  point_xy_t lower_left, upper_right;
  map->bounding_box(gnss_reference, lower_left, upper_right);

  const double diag_length = abs(boost::make_tuple(boost::get<X>(upper_right) - boost::get<X>(lower_left),
                                                boost::get<Y>(upper_right) - boost::get<Y>(lower_left)));

  boost::get<X>(lower_left) = boost::get<X>(lower_left) - margin_percentual * diag_length;
  boost::get<Y>(lower_left) = boost::get<Y>(lower_left) - margin_percentual * diag_length;
  boost::get<X>(upper_right) = boost::get<X>(upper_right) + margin_percentual * diag_length;
  boost::get<Y>(upper_right) = boost::get<Y>(upper_right) + margin_percentual * diag_length;

  // calculate number of horizontal and vertical samples
  const double diff_x = boost::get<X>(upper_right) - boost::get<X>(lower_left);
  const double diff_y = boost::get<Y>(upper_right) - boost::get<Y>(lower_left);

  const std::size_t size_x = std::floor(diff_x / grid_cell_size);
  const std::size_t size_y = std::floor(diff_y / grid_cell_size);


  // --- plot lanelet metric coordinates ---
  const std::string fn2 = filename_without_suffix + "_lanelets.gpldata";
  std::ofstream lanelet_file(fn2.c_str(), std::ios::out);
  for (std::size_t lanelet_index = 0; lanelet_index<map->lanelets().size(); ++lanelet_index)
  {
    lanelet_file << "# Lanelet "<< map->lanelets()[lanelet_index]->id() << " metric point data using reference point (lat, lon): "
                 << boost::get<LAT>(gnss_reference) << ", " << boost::get<LON>(gnss_reference) << std::endl;

    strip_ptr_t left = boost::get<LEFT>(map->lanelets()[lanelet_index]->bounds());
    for (std::size_t i=0; i<left->pts().size(); ++i)
    {
      point_xy_t metric = vec(gnss_reference, left->pts()[i]);
      lanelet_file << boost::get<X>(metric) << "  " << boost::get<Y>(metric) << std::endl;
    }
    // add empty lines to separate linestrips
    lanelet_file << std::endl << std::endl << std::endl;

    strip_ptr_t right = boost::get<RIGHT>(map->lanelets()[lanelet_index]->bounds());
    for (std::size_t i=0; i<right->pts().size(); ++i)
    {
      point_xy_t metric = vec(gnss_reference, right->pts()[i]);
      lanelet_file << boost::get<X>(metric) << "  " << boost::get<Y>(metric) << std::endl;
    }

    // add empty lines to separate linestrips
    lanelet_file << std::endl << std::endl << std::endl;
  }

  lanelet_file << std::endl;
  lanelet_file.close();

  // --- plot distance data ---
  const std::string fn = filename_without_suffix + "_distances.gpldata";
  std::ofstream distance_file(fn.c_str(), std::ios::out);

  distance_file << "# Lanelet Map distance data" << std::endl;

  // sample the bounding box and calculate the signed distances to all lanelets, find the smallest
  for (std::size_t iy=0; iy<size_y; ++iy)
  {
    for (std::size_t ix=0; ix<size_x; ++ix)
    {
      point_xy_t sample;
      boost::get<X>(sample) = boost::get<X>(lower_left) + ix * grid_cell_size;
      boost::get<Y>(sample) = boost::get<Y>(lower_left) + iy * grid_cell_size;
      const point_with_id_t sample_gnss = from_vec(gnss_reference, sample);

      // find the minimum
      double min_sq_av_distance = std::numeric_limits<double>::infinity();
      for (std::size_t i=0; i<map->lanelets().size(); ++i)
      {
        const double d_left  = (boost::get<LEFT>(map->lanelets()[i]->bounds()))->signed_distance(sample_gnss);
        const double d_right = (boost::get<RIGHT>(map->lanelets()[i]->bounds()))->signed_distance(sample_gnss);
        const double sq_av_distance = sq(0.5 * (d_left + d_right));
        // std::cout << "Distance to lanelet " << map->lanelets()[i]->id() << " is " << sq_av_distance << std::endl;
        if (sq_av_distance < min_sq_av_distance)
        {
          // std::cout << " --> this is a new minimum: " << sq_av_distance << " < " << min_sq_av_distance << std::endl;
          min_sq_av_distance = sq_av_distance;
        }
      }
      // std::cout << " == global minimum is " << min_sq_av_distance << std::endl << std::endl;
      distance_file << boost::get<X>(sample) << "  " << boost::get<Y>(sample) << "  " << min_sq_av_distance << std::endl;
    }
  }

  distance_file << std::endl;

  distance_file.close();

  const std::string fn_gnuplot = filename_without_suffix + ".gpl";
  std::ofstream gnuplot_file(fn_gnuplot.c_str(), std::ios::out);

  gnuplot_file << "# Gnuplot file. Draw with gnuplot -p "<< fn_gnuplot << std::endl;
  gnuplot_file << "set mapping cartesian" << std::endl;
  gnuplot_file << "set mouse" << std::endl;
  gnuplot_file << "splot '"<< fn << "' using 1:2:3:3 w p palette , '"<< fn2 <<"' using 1:2:(0) with lines" << std::endl;
  gnuplot_file.close();

  std::cout << "GnuPlot file written to " << fn_gnuplot << std::endl;
}



int main(int argc, char *argv[])
{
  std::string filename = "";
  if (argc > 1)
  {
    filename = std::string(argv[1]);
  }
  else
  {
    std::cerr << "usage: " << argv[0] << " <lanelet-filename>" << std::endl;
    return -1;
  }

  lanelet_map_ptr_t map(new LaneletMap(filename));

  assert(map->lanelets().empty() == false);

  // plot all lanelets separately
  for (std::size_t i=0; i<map->lanelets().size(); ++i)
  {
    std::stringstream fname;
    fname << "lanelet" << std::abs(map->lanelets()[i]->id()) << "_distances";
    plotLaneletDistances(map->lanelets()[i], fname.str());
  }

  // plot the whole map
  const point_with_id_t reference = (boost::get<CENTER>(map->lanelets()[0]->bounds()))->pts()[0];

  std::string fname("lanelet_map_distances");
  std::cout << "Sampling distances for the whole map - this may take a while.." << std::endl;
  plotMapDistances(reference, map, fname);

  return 0;
}

