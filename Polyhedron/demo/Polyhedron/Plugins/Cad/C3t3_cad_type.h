#ifndef CGAL_DEMO_MESH_3_C3T3_CAD_TYPE_H
#define CGAL_DEMO_MESH_3_C3T3_CAD_TYPE_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/cgalMeshDomainWithRationalBezierFeatures.h>

#include <CGAL/make_mesh_3.h>
#include <CGAL/perturb_mesh_3.h>
#include <CGAL/exude_mesh_3.h>

#include <CGAL/IO/File_binary_mesh_3.h>

#include <cgalBrepMeshDomainData.h>

// /////////////////////////////////////////////////////////////////
// Typedef
// /////////////////////////////////////////////////////////////////
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

// /////////////////////////////////////////////////////////////////
// Domains
// /////////////////////////////////////////////////////////////////
typedef cgalBrepMeshDomainData< K > Mesh_domain;
typedef Mesh_domain::Index Index;
typedef Mesh_domain::Construct_initial_points Construct_initial_points;

// Triangulation
typedef CGAL::Compact_mesh_cell_base_3<K, Mesh_domain>    Cell_base;
typedef CGAL::Triangulation_cell_base_with_info_3<int, K, Cell_base> Cell_base_with_info;

// /////////////////////////////////////////////////////////////////
//  Triangulation
// /////////////////////////////////////////////////////////////////

#ifdef CGAL_CONCURRENT_MESH_3
  typedef CGAL::Mesh_triangulation_3<Mesh_domain,
                                     K,
                                     CGAL::Parallel_tag,
                                     CGAL::Default,
                                     Cell_base_with_info>::type Tr;
#else
  typedef CGAL::Mesh_triangulation_3<Mesh_domain,
                                     K,
                                     CGAL::Sequential_tag,
                                     CGAL::Default,
                                     Cell_base_with_info>::type Tr;
#endif

typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;

// // /////////////////////////////////////////////////////////////////
// // Criteria
// // /////////////////////////////////////////////////////////////////
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;
typedef Mesh_criteria::Facet_criteria Facet_criteria;
typedef Mesh_criteria::Cell_criteria Cell_criteria;

typedef Tr::Geom_traits Geom_traits;

typedef Tr::Weighted_point                 Weighted_point;
typedef Tr::Vertex_handle                  Vertex_handle;

#endif // CGAL_DEMO_MESH_3_C3T3_TYPE_H
