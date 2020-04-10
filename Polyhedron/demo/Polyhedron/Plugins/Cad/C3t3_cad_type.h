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
typedef cgalBrepMeshDomainData< K > Mesh_domain_cad;
typedef Mesh_domain_cad::Index Index_cad;
typedef Mesh_domain_cad::Construct_initial_points Construct_initial_points_cad;


// Triangulation
typedef CGAL::Compact_mesh_cell_base_3<K, Mesh_domain_cad>    Cell_base_cad;
typedef CGAL::Triangulation_cell_base_with_info_3<int, K, Cell_base_cad> Cell_base_cad_with_info;

// /////////////////////////////////////////////////////////////////
//  Triangulation
// /////////////////////////////////////////////////////////////////

#ifdef CGAL_CONCURRENT_MESH_3
  typedef CGAL::Mesh_triangulation_3<Mesh_domain_cad,
                                     K,
                                     CGAL::Parallel_tag,
                                     CGAL::Default,
                                     Cell_base_cad_with_info>::type Tr_cad;
#else
  typedef CGAL::Mesh_triangulation_3<Mesh_domain_cad,
                                     K,
                                     CGAL::Sequential_tag,
                                     CGAL::Default,
                                     Cell_base_cad_with_info>::type Tr_cad;
#endif

typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr_cad> C3t3_cad;

// // /////////////////////////////////////////////////////////////////
// // Criteria
// // /////////////////////////////////////////////////////////////////
typedef CGAL::Mesh_criteria_3<Tr_cad> Mesh_criteria_cad;
typedef Mesh_criteria_cad::Facet_criteria Facet_criteria_cad;
typedef Mesh_criteria_cad::Cell_criteria Cell_criteria_cad;

typedef Tr_cad::Geom_traits Geom_traits_cad;

typedef Tr_cad::Weighted_point                 Weighted_point_cad;
typedef Tr_cad::Vertex_handle                  Vertex_handle_cad;

#endif // CGAL_DEMO_MESH_3_C3T3_TYPE_H
