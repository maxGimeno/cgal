// Copyright (c) 2006  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL$
// $Id$
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Fernando Cacciola <fernando.cacciola@geometryfactory.com>
//
#ifndef CGAL_SURFACE_MESH_SIMPLIFICATION_EDGE_COLLAPSE_H
#define CGAL_SURFACE_MESH_SIMPLIFICATION_EDGE_COLLAPSE_H

#include <CGAL/license/Surface_mesh_simplification.h>

#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/Named_function_parameters.h>
#include <CGAL/boost/graph/named_params_helper.h>

#include <CGAL/Surface_mesh_simplification/internal/Common.h>
#include <CGAL/Surface_mesh_simplification/internal/Edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_distance_placement.h>
#include <CGAL/Surface_mesh_simplification/internal/Placement_wrappers.h>

namespace CGAL {
namespace Surface_mesh_simplification {
namespace internal {

template<class TM,
         class GT,
         class ShouldStop,
         class VertexIndexMap,
         class VertexPointMap,
         class HalfedgeIndexMap,
         class EdgeIsConstrainedMap,
         class GetCost,
         class GetPlacement,
         class Visitor>
int edge_collapse(TM& tmesh,
                  const ShouldStop& should_stop,
                  // optional mesh information policies
                  const GT& traits,
                  const VertexIndexMap& vim, // defaults to get(vertex_index, tmesh)
                  const VertexPointMap& vpm, // defaults to get(vertex_point, tmesh)
                  const HalfedgeIndexMap& him, // defaults to get(edge_index, tmesh)
                  const EdgeIsConstrainedMap& ecm, // defaults to No_constrained_edge_map<TM>()
                  // optional strategy policies - defaults to LindstomTurk
                  const GetCost& get_cost,
                  const GetPlacement& get_placement,
                  Visitor visitor)
{
  typedef EdgeCollapse<TM, GT, ShouldStop,
                       VertexIndexMap, VertexPointMap, HalfedgeIndexMap, EdgeIsConstrainedMap,
                       GetCost, GetPlacement, Visitor> Algorithm;

  Algorithm algorithm(tmesh, traits, should_stop, vim, vpm, him, ecm, get_cost, get_placement, visitor);

  return algorithm.run();
}

struct Dummy_visitor
{
  template<class TM>
  void OnStarted(TM&) const {}
  template<class TM>
  void OnFinished(TM&) const {}
  template<class Profile>
  void OnStopConditionReached(const Profile&) const {}
  template<class Profile, class OFT>
  void OnCollected(const Profile&, const OFT&) const {}
  template<class Profile, class OFT, class Size_type>
  void OnSelected(const Profile&, const OFT&, Size_type, Size_type) const {}
  template<class Profile, class OPoint>
  void OnCollapsing(const Profile&, const OPoint&) const {}
  template<class Profile, class VH>
  void OnCollapsed(const Profile&, VH) const {}
  template<class Profile>
  void OnNonCollapsable(const Profile&) const {}
};

//To detect if a Placement type is already constrained
BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(Has_nested_type_constrained_tag, Constrained_tag, false)

} // namespace internal

template<class TM, class ShouldStop, class NamedParameters>
int edge_collapse(TM& tmesh,
                  const ShouldStop& should_stop,
                  const NamedParameters& np)
{
  using parameters::choose_parameter;
  using parameters::get_parameter;
  
  typedef typename GetGeomTraits<TM, NamedParameters>::type         Geom_traits;
  
  typedef typename internal_np::Lookup_named_param_def <
      internal_np::edge_is_constrained_t,
      NamedParameters, internal_np::Param_not_found > ::type        Cmo_type;
  
  typedef typename internal_np::Lookup_named_param_def <
      internal_np::max_normal_angle_change_t,
      NamedParameters, internal_np::Param_not_found > ::type        Angle_param_type;
  
  typedef typename internal_np::Lookup_named_param_def <
      internal_np::max_input_dist_placement_t,
      NamedParameters, internal_np::Param_not_found > ::type        Dist_param_type;
  
  typedef typename internal_np::Lookup_named_param_def <
      internal_np::edge_is_constrained_t,
      NamedParameters, No_constrained_edge_map<TM> > ::type         Fcm_type;
  
  typedef typename internal_np::Lookup_named_param_def <
      internal_np::get_placement_policy_t,
      NamedParameters, LindstromTurk_placement<TM> > ::type         Placement_type;
  
  typedef typename internal::GetPlacementType<Placement_type,
      Cmo_type>                                                     Tmp_placement;
  
  typedef typename internal::HasAngleBound<typename Tmp_placement::type,
      Angle_param_type>                                             AnglePlacement;
  
  typedef typename internal::HasDistBound<typename AnglePlacement::type,
      Geom_traits, Dist_param_type>                                 FinalPlacement;
  
  Cmo_type c_map
      = get_parameter(np, internal_np::edge_is_constrained);
  
  Placement_type placement
      = choose_parameter<LindstromTurk_placement<TM> >(get_parameter(np, internal_np::get_placement_policy));
  
  bool do_constrain = choose_parameter(get_parameter(np, internal_np::constrain_geometry),
                                       true);
  
  typename Tmp_placement::type tmp_placement = Tmp_placement::get_placement(c_map, placement,
                                                                            !internal::Has_nested_type_constrained_tag<Placement_type>::value && do_constrain);
  
  double max_angle = choose_parameter(get_parameter(np, internal_np::max_normal_angle_change),
                                      CGAL_PI);
  double max_dist = choose_parameter(get_parameter(np, internal_np::max_input_dist_placement),
                                      -1);//default is ignored, so any value here is fine.
  
  typename AnglePlacement::type angle_placement = AnglePlacement::get_placement(max_angle, tmp_placement);
  typename FinalPlacement::type final_placement = FinalPlacement::get_placement(max_dist, angle_placement);
  Fcm_type final_map = choose_parameter<No_constrained_edge_map<TM> >(get_parameter(np, internal_np::edge_is_constrained));
  
  return internal::edge_collapse(tmesh, should_stop,
                                 choose_parameter<Geom_traits>(get_parameter(np, internal_np::geom_traits)),
                                 CGAL::get_initialized_vertex_index_map(tmesh, np),
                                 choose_parameter(get_parameter(np, internal_np::vertex_point),
                                                  get_property_map(vertex_point, tmesh)),
                                 CGAL::get_initialized_halfedge_index_map(tmesh, np),
                                 final_map,
                                 choose_parameter<LindstromTurk_cost<TM> >(get_parameter(np, internal_np::get_cost_policy)),
                                 final_placement,
                                 choose_parameter<internal::Dummy_visitor>(get_parameter(np, internal_np::graph_visitor)));
}

template<class TM, class ShouldStop>
int edge_collapse(TM& tmesh, const ShouldStop& should_stop)
{
  return edge_collapse(tmesh, should_stop, CGAL::parameters::all_default());
}

} // namespace Surface_mesh_simplification
} // namespace CGAL

#endif // CGAL_SURFACE_MESH_SIMPLIFICATION_EDGE_COLLAPSE_H
