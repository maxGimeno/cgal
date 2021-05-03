// Copyright (c) 2009-2014 INRIA Sophia-Antipolis (France).
// Copyright (c) 2010-2013 GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL$
// $Id$
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     :  Mael Rouxel-Labb�, Maxime Gimeno, Jane Tournois
//
//******************************************************************************
// File Description :
//******************************************************************************

#ifndef CGAL_MDS_3_TETRAHEDRON_SOUP_TO_C3T3_H
#define CGAL_MDS_3_TETRAHEDRON_SOUP_TO_C3T3_H

#include <CGAL/license/MDS_3.h>

#include <CGAL/MDS_3/tet_soup_to_c3t3.h>
#include <CGAL/boost/graph/named_params_helper.h>

#include <vector>
#include <array>
#include <map>

namespace CGAL {

  /** \ingroup PkgMDS3Functions
   * builds a 3D triangulation from a soup of tetrahedra.
   *
   * @tparam TetrahedronRange
   * @tparam Triangulation model of
   * @param tets
   * @param tr
   *
   * @pre convex volume
  */
  template<typename TetrahedronRange, typename Triangulation>
  void tetrahedron_soup_to_triangulation_3(const TetrahedronRange& tets,
                                           Triangulation& tr)
  {
    typedef Triangulation              Tr;
    typedef typename Tr::Point         Point;

    std::vector<Point> points;
    std::vector<std::array<int, 5> > finite_cells;
    std::map<std::array<int, 3>, typename Tr::Cell::Surface_patch_index> border_facets;
    std::map<Point, int> p2i;

    for (typename TetrahedronRange::value_type tet : tets)
    {
      CGAL_assertion(tet.orientation() == CGAL::POSITIVE);
      std::array<int, 5> cell;

      for (int i = 0; i < 4; ++i)
      {
        const Point& pi = tet[i];
        if (p2i.find(pi) == p2i.end())
        {
          points.push_back(pi);
          p2i.insert(std::make_pair(pi, points.size() - 1));
          cell[i] = static_cast<int>(points.size() - 1);
        }
        else
          cell[i] = p2i.at(pi);
      }
      cell[4] = 1;

      CGAL_assertion(CGAL::orientation(points[cell[0]],
        points[cell[1]], points[cell[2]], points[cell[3]]) == CGAL::POSITIVE);

      finite_cells.push_back(cell);
    }

    CGAL::MDS_3::build_triangulation(tr, points, finite_cells, border_facets);
  }

  /** \ingroup PkgMDS3Functions
  * builds a 3D triangulation from a soup of tetrahedra.
  *
  * @tparam PointRange  a model of the concept `RandomAccessContainer`
  * whose value type is the point type
  * @tparam TetrahedronRange a model of the concept `RandomAccessContainer` whose
  * value type is a model of the concept `RandomAccessContainer` whose value type is `std::size_t`
  *
  * @param points points of the soup of tetrahedra
  * @param tets each element in the range describes a tetrahedron using the indices of the points
  * in `points` (indices 0 to 3), and the associated `Subdomain_index` (index 4)
  * @param tr the 3D triangulation to be built
  * @param np an optional sequence of \ref mds3_namedparameters "Named Parameters" among the ones listed below
  *
  * \cgalNamedParamsBegin
  *   \cgalParamNBegin{surface_facets}
  *     \cgalParamDescription{each element in the range describes a surface facet using the indices of points
  *       in `points` (indices 0 to 2), and the associated `Surface_patch_index` (index 3)}
  *     \cgalParamType{a class model of `AssociativeContainer`
  *                    whose key type is model of `RandomAccessContainer`
  *                    and mapped type is `Tr::Cell::Surface_patch_index`}
  *     \cgalParamDefault{An empty `std::map<std::array<int, 3>, typename Tr::Cell::Surface_patch_index>`}
  *     \cgalParamExtra{If this parameter is omitted, an internal property map for `CGAL::vertex_point_t`
  *                     must be available in `PolygonMesh`.}
  *   \cgalParamNEnd
  * \cgalNamedParamsEnd
  *
  * @pre `points` contains each point only once
  *
  * @sa `CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh()`
  */
  template<typename PointRange,
           typename TetrahedronRange,
           typename Triangulation,
           typename NamedParameters>
  void tetrahedron_soup_to_triangulation_3(const PointRange& points,
                                           const TetrahedronRange& tets,
                                           Triangulation& tr,
                                           const NamedParameters& np)
  {
    using parameters::choose_parameter;
    using parameters::get_parameter;

    std::map<std::array<int, 3>,
             typename Triangulation::Cell::Surface_patch_index> empty_map;
    auto facets = choose_parameter(get_parameter(np, internal_np::surface_facets), empty_map);

    CGAL::MDS_3::build_triangulation(tr, points, tets, facets);
  }

  template<typename PointRange,
           typename TetrahedronRange,
           typename Triangulation>
  void tetrahedron_soup_to_triangulation_3(const PointRange& points,
                                           const TetrahedronRange& tets,
                                           Triangulation& tr)
  {
    tetrahedron_soup_to_triangulation_3(points, tets, tr, parameters::all_default());
  }

} //namespace CGAL


#endif // CGAL_MDS_3_TETRAHEDRON_SOUP_TO_C3T3_H
