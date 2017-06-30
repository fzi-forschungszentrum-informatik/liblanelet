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

void plotPolygonData(point_with_id_t gnss_reference, lanelet_ptr_t lanelet, const std::string& filename_without_suffix)
{
  // -- data formatting --

  const std::string fn = filename_without_suffix + "_polygon.gpldata";

  std::ofstream polygon_file(fn.c_str(), std::ios::out);


  polygon_file << "# Lanelet "<< lanelet->id() << " polygon data" << std::endl;
  Polygon poly(lanelet); // have to deep copy because polygon is only forwarded to lanelet
  assert(poly.vertices().empty() == false && "The polygon is empty");

  for (std::size_t i=0; i<poly.vertices().size(); ++i)
  {
    polygon_file << boost::get<X>(poly.vertices()[i]) << "  "
                 << boost::get<Y>(poly.vertices()[i]) << std::endl;
  }
  // add the first point again to close the drawing
  polygon_file << boost::get<X>(poly.vertices()[0]) << "  "
               << boost::get<Y>(poly.vertices()[0]) << std::endl;
  polygon_file << std::endl;

  polygon_file.close();

  const std::string fn_gnuplot = filename_without_suffix + ".gpl";
  std::ofstream gnuplot_file(fn_gnuplot.c_str(), std::ios::out);

  gnuplot_file << "# Gnuplot file. Draw with gnuplot -p "<< fn_gnuplot << std::endl;
  gnuplot_file << "set mapping cartesian" << std::endl;
  gnuplot_file << "set mouse" << std::endl;
  gnuplot_file << "plot '"<< fn << "' using 1:2 w lp" << std::endl;
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

  point_with_id_t reference = (boost::get<CENTER>(map->lanelets()[0]->bounds()))->pts()[0];
  for (std::size_t i=0; i<map->lanelets().size(); ++i)
  {
    std::stringstream fname;
    fname << "lanelet" << std::abs(map->lanelets()[i]->id()) << "_polygon";
    plotPolygonData(reference, map->lanelets()[i], fname.str());
  }

  return 0;
}

