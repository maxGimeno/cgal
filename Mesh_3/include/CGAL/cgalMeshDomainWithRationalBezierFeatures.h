 // Copyright (c) 2009-2010 INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
//
//
// Author(s)     : Come Le Breton, Laurent Rineau, Jane Tournois
//
//******************************************************************************
// File Description :
//
//******************************************************************************

#pragma once

#include <CGAL/iterator.h>
#include <CGAL/enum.h>
#include <CGAL/number_utils.h>
#include <CGAL/is_streamable.h>
#include <CGAL/Real_timer.h>

#include <vector>
#include <set>
#include <map>
#include <boost/next_prior.hpp> // for boost::prior and boost::next
#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include <dtkContinuousGeometryUtils>
#include <dtkRationalBezierCurve>
#include <dtkNurbsCurve>
#include <dtkNurbsCurveIntersect>

#include <tuple>
#include <unordered_set>

namespace CGAL {
    /**
     * @class cgalMeshDomainWithRationalBezierFeatures
     *
     *
     */
    template < typename MeshDomain >
    class cgalMeshDomainWithRationalBezierFeatures : public MeshDomain
    {
        typedef MeshDomain Base;

    public:
        // Index types
        typedef typename Base::Index    Index;

        typedef typename Base::Surface_patch_index Surface_patch_index;
        typedef typename Base::Construct_initial_points Construct_initial_points;
        typedef int Curve_segment_index;
        typedef int Corner_index;
        typedef int Curve_index;

        typedef typename Base::R         Gt;
        typedef Gt                       R;
        typedef typename Base::Point_3   Point_3;
        typedef typename Gt::FT          FT;

        typedef CGAL::Tag_true           Has_features;


#ifndef CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES
        template <typename ... T>
        cgalMeshDomainWithRationalBezierFeatures(const T& ...t)
            : Base(t...)
            , current_corner_index_(1)
            , current_curve_index_(1)
            , current_nurbs_curve_index_(0)
        {}
#else
        /// Constructors
        /// Call the base class constructor
        cgalMeshDomainWithRationalBezierFeatures()
            : Base()
            , current_corner_index_(1)
            , current_curve_index_(1)
            , current_nurbs_curve_index_(0)
        {}

        template <typename T1>
        cgalMeshDomainWithRationalBezierFeatures(const T1& o1)
            : Base(o1)
            , current_corner_index_(1)
            , current_curve_index_(1)
            , current_nurbs_curve_index_(0)
        {}
        template <typename T1, typename T2>
        cgalMeshDomainWithRationalBezierFeatures(const T1& o1, const T2& o2)
            : Base(o1, o2)
            , current_corner_index_(1)
            , current_curve_index_(1)
            , current_nurbs_curve_index_(0)
        {}
        template <typename T1, typename T2, typename T3>
        cgalMeshDomainWithRationalBezierFeatures(const T1& o1, const T2& o2,
                                                 const T3& o3)
            : Base(o1, o2, o3)
            , current_corner_index_(1)
            , current_curve_index_(1)
            , current_nurbs_curve_index_(0)
        {}
        template <typename T1, typename T2, typename T3, typename T4>
        cgalMeshDomainWithRationalBezierFeatures(const T1& o1, const T2& o2,
                                                 const T3& o3, const T4& o4)
            : Base(o1, o2, o3, o4)
            , current_corner_index_(1)
            , current_curve_index_(1)
            , current_nurbs_curve_index_(0)
            {}
#endif

        /// Destructor
        ~cgalMeshDomainWithRationalBezierFeatures()
        {
            for(auto edge = edges_.begin(); edge != edges_.end(); ++edge) {
                delete (edge->second).first;
                delete (edge->second).second;
            }
        }

        /// OutputIterator value type is std::pair<Corner_index, Point_3>
        template <typename OutputIterator>
        OutputIterator get_corners(OutputIterator out) const;

        /// OutputIterator value type is CGAL::cpp11::tuple<Curve_segment_index,
        /// Corner_index, Corner_index >
        template <typename OutputIterator>
        OutputIterator get_curve_segments(OutputIterator out) const;

        const Curve_index& get_curve_index(const Curve_segment_index& curve_segment_index) const {
            return curve_segment_index_to_curve_index_.at(curve_segment_index);
        };

        ///Returns the parameters of the corner given by /c corner_index on the curve given by /c curve_index
        FT get_corner_parameter_on_curve(const Corner_index& corner_index, const Curve_segment_index& curve_index) const;

        /// Returns a maximal error bound on the distance bewteen the cord linking C(p) to C(q), to the curve from C(p) to C(q)
        /// \c p : parameter of C(p) in C parameter space
        /// \c q : parameter of C(q) in C parameter space
        /// \c curve_index
        FT error_bound_cord_to_curve(double p, double q, const Curve_segment_index& curve_index) const;

        /// Returns true if the spheres located at p on curve_index_p and q on curve_index_q covers the three nurbs curves
        /// Returns false if the spheres do not cover the curves, or if they do not belong to the same nurbs curve
        /// \c p : parameter
        /// \c q : parameter
        /// \c curve_index_p
        /// \c curve_index_q
        bool curves_cover(const Curve_segment_index& curve_segment_index_p, FT p, Point_3 center_p, FT radius_p, const Curve_segment_index& curve_segment_index_q, FT q, Point_3 center_q, FT radius_q) const;

        /// Construct a point on curve \c curve_index at parameter p
        /// of \c starting_point
        Point_3 construct_point_on_curve_segment(double p, const Curve_segment_index& curve_index) const;

        /// Returns the sign of the orientation of p,q,r along curve segment
        /// of index \c index
        /// \c p : parameter of C(p) in C parameter space
        /// \c q : parameter of C(q) in C parameter space
        /// \c r : parameter of C(r) in C parameter space
        CGAL::Sign distance_sign_along_cycle(double p, double q, double r, const Curve_segment_index& index) const;

        /// Returns true if curve \c curve_index is a cycle. The point is ignored.
        bool is_cycle(const Point_3&, const Curve_segment_index& index) const;
        bool is_cycle(const Curve_segment_index& index) const;

        /// Returns an Index from a Curve_segment_index
        Index index_from_curve_segment_index(const Curve_segment_index& index) const
        { return Index(index); }

        /// Returns an Curve_segment_index from an Index
        Curve_segment_index curve_segment_index(const Index& index) const
        { return boost::get<Curve_segment_index>(index); }

        /// Returns an Index from a Corner_index
        Index index_from_corner_index(const Corner_index& index) const
        { return Index(index); }

        /// Returns an Corner_index from an Index
        Corner_index corner_index(const Index& index) const
        { return boost::get<Corner_index>(index); }

        /// Insert a bunch of edges into domain
        ///   + InputIterator must be of type dtkRationalBezierCurve
        //    + IndicesOutputIterator is an output iterator of value_type equal
        ///   to Curve_segment_index
        template <typename InputIterator, typename IndicesOutputIterator>
        IndicesOutputIterator
        add_features(InputIterator first, InputIterator last,
                     IndicesOutputIterator out /*= CGAL::Emptyset_iterator()*/);

        template <typename InputIterator>
        void
        add_features(InputIterator first, InputIterator last)
        { add_features(first, last, CGAL::Emptyset_iterator()); }

  template <typename IndicesOutputIterator>
  IndicesOutputIterator
  get_incidences(Curve_segment_index id, IndicesOutputIterator out) const;

  template <typename IndicesOutputIterator>
  IndicesOutputIterator
  get_corner_incidences(Corner_index id, IndicesOutputIterator out) const;

  template <typename IndicesOutputIterator>
  IndicesOutputIterator
  get_corner_incident_curves(Corner_index id, IndicesOutputIterator out) const;

  typedef std::set<Surface_patch_index> Surface_patch_index_set;

  const Surface_patch_index_set&
  get_incidences(Curve_segment_index id) const;

  /// Returns Index associated to p (p must be the coordinates of a corner
  /// point)
        Index point_corner_index(const Point_3& p) const;

        //Returns Point_3 the point associated to the Corner_index p_corner_index
        Point_3 corner_point_from_index(Corner_index p_corner_index) const;

    private:
        Corner_index register_corner(const Point_3& p, const Curve_segment_index& index);
        void compute_corners_incidences();

        /// Returns the sign of the geodesic distance between \c p and \c q
        /// Precondition: index is not a cycle
        CGAL::Sign distance_sign(double p, double q,
                                 const Curve_segment_index& index) const;

    private:
        typedef std::map<Point_3, Corner_index> Corners;
        typedef std::map<Corner_index, Point_3> Corners_reverse;


        typedef std::map< Curve_segment_index, std::pair< dtkRationalBezierCurve *, double * > > Edges;
        typedef std::map< Curve_segment_index,  Curve_index > Curve_segment_index_to_curve_index;
        typedef std::map< Curve_index, std::tuple< dtkNurbsCurveIntersect *, dtkNurbsCurveIntersect *, dtkNurbsCurveIntersect* > > Nurbs_edge_intersects;

        typedef std::map< Curve_segment_index, Surface_patch_index_set > Edges_incidences;
        typedef std::map< Corner_index, std::set<Curve_segment_index> > Corners_tmp_incidences;
        typedef std::map< Corner_index, Surface_patch_index_set > Corners_incidences;

        typedef std::map< std::pair<Corner_index, Curve_segment_index>, double > Corners_parameters;

        Corners corners_;
        Corners_reverse corners_reverse_;
        Corners_tmp_incidences corners_tmp_incidences_;
        Corner_index current_corner_index_;
        Corners_incidences corners_incidences_;
        Corners_parameters corners_parameters;

        Edges edges_;
        Curve_segment_index current_curve_index_;
        Nurbs_edge_intersects nurbs_edge_intersects_;
        Curve_index current_nurbs_curve_index_;
        Curve_segment_index_to_curve_index curve_segment_index_to_curve_index_;

        Edges_incidences edges_incidences_;

    public:
        typedef std::set< Surface_patch_index > Set_of_patch_ids;
        typedef std::map< Point_3, Set_of_patch_ids > Corners_incidence_map;

    private:
        Corners_incidence_map corners_incidence_map_;

    public:
        const Corners_incidence_map& corners_incidences_map() const
        { return corners_incidence_map_; }

        Curve_segment_index maximal_curve_segment_index() const {
            if(edges_incidences_.empty()) return Curve_segment_index();
            return boost::prior(edges_incidences_.end())->first;
        }

    private:
        // Disabled copy constructor & assignment operator
        typedef cgalMeshDomainWithRationalBezierFeatures Self;
        cgalMeshDomainWithRationalBezierFeatures(const Self& src);
        Self& operator=(const Self& src);

    };  // end class cgalMeshDomainWithRationalBezierFeatures


    template <class MD_>
    template <typename OutputIterator>
    OutputIterator
    cgalMeshDomainWithRationalBezierFeatures<MD_>::
    get_corners(OutputIterator out) const
    {
        for ( typename Corners::const_iterator
                  cit = corners_.begin(), end = corners_.end() ; cit != end ; ++cit )
            {
                *out++ = std::make_pair(cit->second,cit->first);
            }

        return out;
    }

    template <class MD_>
    template <typename OutputIterator>
    OutputIterator
    cgalMeshDomainWithRationalBezierFeatures<MD_>::
    get_curve_segments(OutputIterator out) const
    {
        dtkContinuousGeometryPrimitives::Point_3 eval_point(0., 0., 0.);
        dtkContinuousGeometryPrimitives::Point_3 first_point(0., 0., 0.);
        dtkContinuousGeometryPrimitives::Point_3 last_point(0., 0., 0.);
        for ( typename Edges::const_iterator
                  eit = edges_.begin(), end = edges_.end() ; eit != end ; ++eit )
            {
                // ///////////////////////////////////////////////////////////////////
                // Checks that the last control points is not equal to the first one
                // ///////////////////////////////////////////////////////////////////
                eit->second.first->controlPoint(0, first_point.data());
                eit->second.first->controlPoint(eit->second.first->degree(), last_point.data());

                Corner_index p_index, q_index;
                //TODO Check distance instead...
                //Evaluate and compare to stored corners
                eit->second.first->evaluatePoint(0., eval_point.data());
                p_index = boost::get<Corner_index>(point_corner_index(Point_3(eval_point[0], eval_point[1], eval_point[2])));
                eit->second.first->evaluatePoint(1., eval_point.data());
                q_index = boost::get<Corner_index>(point_corner_index(Point_3(eval_point[0], eval_point[1], eval_point[2])));

                *out++ = CGAL::cpp11::make_tuple(eit->first,
                                                 p_index,
                                                 q_index);
            }
        return out;
    }

    template <class MD_>
    typename cgalMeshDomainWithRationalBezierFeatures<MD_>::FT
    cgalMeshDomainWithRationalBezierFeatures<MD_>::get_corner_parameter_on_curve(const Corner_index& corner_index, const Curve_segment_index& curve_index) const
    {
        return corners_parameters.at(std::make_pair(corner_index, curve_index));
    }

    template <class MD_>
    typename cgalMeshDomainWithRationalBezierFeatures<MD_>::Index
    cgalMeshDomainWithRationalBezierFeatures<MD_>::
    point_corner_index(const Point_3& p) const
    {
        bool found = false;
        for(auto it = corners_.begin(); it != corners_.end(); ++it) {
            if(CGAL::squared_distance(p, it->first) < 1e-7) {
            return it->second;
        }
    }
    CGAL_assertion(false);
    return Index();
}

template <class MD_>
typename cgalMeshDomainWithRationalBezierFeatures<MD_>::Point_3
cgalMeshDomainWithRationalBezierFeatures<MD_>::
construct_point_on_curve_segment(double p, const Curve_segment_index& curve_index) const
{
    // ///////////////////////////////////////////////////////////////////
    // Recovers the edge
    // ///////////////////////////////////////////////////////////////////
    typename Edges::const_iterator eit = edges_.find(curve_index);
    CGAL_assertion(eit != edges_.end());
    dtkContinuousGeometryPrimitives::Point_3 eval_point(0., 0., 0.);
    eit->second.first->evaluatePoint(p, eval_point.data());
    return Point_3(eval_point[0], eval_point[1], eval_point[2]);
}

template <class MD_>
typename cgalMeshDomainWithRationalBezierFeatures<MD_>::FT
cgalMeshDomainWithRationalBezierFeatures<MD_>::error_bound_cord_to_curve(double p, double q, const Curve_segment_index& curve_index) const
{
    // ///////////////////////////////////////////////////////////////////
    // Recovers the edge
    // ///////////////////////////////////////////////////////////////////
    typename Edges::const_iterator eit = edges_.find(curve_index);
    CGAL_assertion(eit != edges_.end());
    // ///////////////////////////////////////////////////////////////////
    // Clip the curve at p and q, recovers the middle bezier curve and compute
    // ///////////////////////////////////////////////////////////////////
    std::vector< double > splitting_parameters(2);
    splitting_parameters[0] = p;
    splitting_parameters[1] = q;
    std::list< dtkRationalBezierCurve * > split_curves;
    eit->second.first->split(split_curves, splitting_parameters);

    return dtkContinuousGeometryTools::convexHullApproximationError(*(*(std::next(split_curves.begin()))));
}

template <class MD_>
bool
cgalMeshDomainWithRationalBezierFeatures<MD_>::
curves_cover(const Curve_segment_index& curve_segment_index_p, FT p, Point_3 center_p, FT radius_p, const Curve_segment_index& curve_segment_index_q, FT q, Point_3 center_q, FT radius_q) const
{
    // ///////////////////////////////////////////////////////////////////
    // Recovers the Curve_index for both Curve_segment_index
    // ///////////////////////////////////////////////////////////////////
    Curve_index curve_index_p = curve_segment_index_to_curve_index_.at(curve_segment_index_p);
    Curve_index curve_index_q = curve_segment_index_to_curve_index_.at(curve_segment_index_q);
    if(curve_index_p != curve_index_q) {
        std::cerr << "The curve segment index do not belong to the same curve... weird." << std::endl;
        return false;
    }
    // ///////////////////////////////////////////////////////////////////
    // Recovers the parameters along the nurbs curve
    // ///////////////////////////////////////////////////////////////////
    std::pair< dtkRationalBezierCurve *, double * > edge_p = edges_.at(curve_index_p);
    std::pair< dtkRationalBezierCurve *, double * > edge_q = edges_.at(curve_index_p);
    FT p_edge = edge_p.second[0] + p * (edge_p.second[1] - edge_p.second[0]);
    FT q_edge = edge_q.second[0] + q * (edge_q.second[1] - edge_q.second[0]);

    // ///////////////////////////////////////////////////////////////////
    // Creates the spheres for the intersections
    // ///////////////////////////////////////////////////////////////////
    dtkContinuousGeometryPrimitives::Sphere_3 p_sphere(dtkContinuousGeometryPrimitives::Point_3(center_p.x(), center_p.y(), center_p.z()), radius_p);
    dtkContinuousGeometryPrimitives::Sphere_3 q_sphere(dtkContinuousGeometryPrimitives::Point_3(center_q.x(), center_q.y(), center_q.z()), radius_q);
    std::tuple< dtkNurbsCurveIntersect *, dtkNurbsCurveIntersect *, dtkNurbsCurveIntersect * > intersects = nurbs_edge_intersects_.at(curve_index_p);

    // ///////////////////////////////////////////////////////////////////
    // Checks for the curve "0" first
    // ///////////////////////////////////////////////////////////////////
    std::list< std::pair< double, double > > intervals_p;
    std::list< std::pair< double, double > > intervals_q;
    std::get<0>(intersects)->intersect(intervals_p, p_sphere);
    std::get<0>(intersects)->intersect(intervals_q, q_sphere);

    // ///////////////////////////////////////////////////////////////////
    // Check if some intervals overlap (it doesnt guarantee anything...)
    // ///////////////////////////////////////////////////////////////////
    bool overlap = false;
    for(auto inter_p = intervals_p.begin(); inter_p != intervals_p.end(); ++inter_p) {
        for(auto inter_q = intervals_q.begin(); inter_q != intervals_q.end(); ++inter_q) {
            if ((*inter_p).second >= (*inter_q).first || (*inter_p).first <= (*inter_q).second) {
                overlap = true;
                break;
            }
        }
    }
    if (overlap == false) {
        return false;
    }
    // ///////////////////////////////////////////////////////////////////
    // Checks for the curve "2"
    // ///////////////////////////////////////////////////////////////////
    intervals_p.clear();
    intervals_q.clear();
    std::get<2>(intersects)->intersect(intervals_p, p_sphere);
    std::get<2>(intersects)->intersect(intervals_q, q_sphere);
    for(auto inter_p = intervals_p.begin(); inter_p != intervals_p.end(); ++inter_p) {
        for(auto inter_q = intervals_q.begin(); inter_q != intervals_q.end(); ++inter_q) {
            if ((*inter_p).second >= (*inter_q).first || (*inter_p).first <= (*inter_q).second) {
                overlap = true;
                break;
            }
        }
    }
    if (overlap == false) {
        return false;
    } else {
        return true;
    }
}

template <class MD_>
template <typename InputIterator, typename IndicesOutputIterator>
IndicesOutputIterator
cgalMeshDomainWithRationalBezierFeatures<MD_>::
add_features(InputIterator first, InputIterator last,
             IndicesOutputIterator indices_out)
{
    // ///////////////////////////////////////////////////////////////////
    // Recovers three NURBS curves as feature,
    // Decompose the middle one into rational Bezier curves, and stores the rational bezier curves in edges_
    // ///////////////////////////////////////////////////////////////////
    std::map< Corner_index, Point_3 > registered_corners;
    // Insert one edge for each element

    while ( first != last ) {
        ++current_nurbs_curve_index_;
        dtkNurbsCurveIntersect * intersect_0 = new dtkNurbsCurveIntersect(*std::get<0>(*first));
        dtkNurbsCurveIntersect * intersect_1 = new dtkNurbsCurveIntersect(*std::get<1>(*first));
        dtkNurbsCurveIntersect * intersect_2 = new dtkNurbsCurveIntersect(*std::get<2>(*first));
        nurbs_edge_intersects_.insert(std::make_pair(current_nurbs_curve_index_, std::make_tuple(intersect_0, intersect_1, intersect_2)));
        // ///////////////////////////////////////////////////////////////////
        // Decompose the middle nurbs curve to rational bezier curves
        // ///////////////////////////////////////////////////////////////////
        std::vector< std::pair< dtkRationalBezierCurve *, double * > > rational_bezier_curves;
        std::cerr << "Before bezier decomposition" << std::endl;
        std::get<1>(*first)->decomposeToRationalBezierCurves(rational_bezier_curves);
        std::cerr << "After bezier decomposition" << std::endl;
        for(auto rbc = rational_bezier_curves.begin(); rbc != rational_bezier_curves.end(); ++rbc) {
            //Starts at 1 for the first curve
            ++current_curve_index_;
            const Curve_segment_index curve_index = current_curve_index_;

            // ///////////////////////////////////////////////////////////////////
            // Creates a mapping from rational bezier curve to initial nurbs curve
            // ///////////////////////////////////////////////////////////////////
            curve_segment_index_to_curve_index_.insert(std::make_pair(current_curve_index_, current_nurbs_curve_index_));

            // ///////////////////////////////////////////////////////////////////
            // Computes the first and last tips of the edge (the corners)
            // ///////////////////////////////////////////////////////////////////
            dtkContinuousGeometryPrimitives::Point_3 first_point(0., 0., 0.);
            dtkContinuousGeometryPrimitives::Point_3 last_point(0., 0., 0.);
            rbc->first->controlPoint(0, first_point.data());
            rbc->first->controlPoint(rbc->first->degree(), last_point.data());
            Point_3 first_cgal(first_point[0], first_point[1], first_point[2]);
            Point_3 last_cgal(last_point[0], last_point[1], last_point[2]);

            ///////////////////////////////////////////////////////////////////
            // Checks that the corner was not already inserted
            // If not, insert it and register it
            ///////////////////////////////////////////////////////////////////
            bool already_inserted = false;
            Corner_index already_inserted_corner;
            for(auto it = registered_corners.begin(); it != registered_corners.end(); ++it) {
                if(CGAL::squared_distance(first_cgal, it->second) < 1e-7) {
                    already_inserted = true;
                    already_inserted_corner = it->first;
                    double w = 0.;
                    (rbc->first)->weight(0, &w);
                    double *new_point = new double[4];
                    new_point[0] = it->second[0];
                    new_point[1] = it->second[1];
                    new_point[2] = it->second[2];
                    new_point[3] = w;
                    (rbc->first)->setWeightedControlPoint(0, &new_point[0]);
                    delete[] new_point;
                    break;

                }
            }
            if (already_inserted) {
                register_corner(registered_corners[already_inserted_corner], curve_index);// for incidences updating
                corners_parameters.insert(std::make_pair(std::make_pair(already_inserted_corner, curve_index), 0.));
            } else {
                Corner_index corner_index = register_corner(first_cgal, curve_index);
                registered_corners.insert(std::make_pair(corner_index, first_cgal));
                //Stores the parameters of the corner on the curve identifid by curve index
                corners_parameters.insert(std::make_pair(std::make_pair(corner_index, curve_index), 0.));
            }

            ///////////////////////////////////////////////////////////////////
            //Same for the end control point
            ///////////////////////////////////////////////////////////////////
            already_inserted = false;
            for(auto it = registered_corners.begin(); it != registered_corners.end(); ++it) {
                if(CGAL::squared_distance(last_cgal, it->second) < 1e-7) {
                    already_inserted = true;
                    already_inserted_corner = it->first;
                    double w = 0.;
                    rbc->first->weight(rbc->first->degree(), &w);
                    double *new_point = new double[4];
                    new_point[0] = it->second[0];
                    new_point[1] = it->second[1];
                    new_point[2] = it->second[2];
                    new_point[3] = w;
                    rbc->first->setWeightedControlPoint(rbc->first->degree(), &new_point[0]);
                    delete[] new_point;
                    break;
                }
            }
            if (already_inserted) {
                register_corner(registered_corners[already_inserted_corner], curve_index);// for incidences updating
                corners_parameters.insert(std::make_pair(std::make_pair(already_inserted_corner, curve_index), 1.));
            } else {
                Corner_index corner_index = register_corner(last_cgal, curve_index);
                registered_corners.insert(std::make_pair(corner_index, last_cgal));
                //Stores the parameters of the corner on the curve identifid by curve index
                corners_parameters.insert(std::make_pair(std::make_pair(corner_index, curve_index), 1.));
            }

            *indices_out++ = curve_index;

            //Create a new rational bezier curve
            std::pair<typename Edges::iterator,bool> insertion = edges_.insert(std::make_pair(curve_index, *rbc));
        }
        ++first;
    }
    compute_corners_incidences();
    return indices_out;
}

template <class MD_>
void
cgalMeshDomainWithRationalBezierFeatures<MD_>::
compute_corners_incidences()
{
  for(typename Corners::iterator
        cit = corners_.begin(), end = corners_.end();
      cit != end;)//the loop variable is incremented in the  body
  {
    const Corner_index id = cit->second;

    const typename Corners_tmp_incidences::mapped_type&
      corner_tmp_incidences = corners_tmp_incidences_[id];

    //If the corner is incident to only one curve, and that curve is a
    //cycle, then remove the corner from the set.
    if(corner_tmp_incidences.size() == 1 &&
       is_cycle(*corner_tmp_incidences.begin()))
    {
      typename Corners::iterator to_erase = cit;
      ++cit;
      corners_.erase(to_erase);
      continue;
    }

    Surface_patch_index_set& incidences = corners_incidences_[id];
    //That should be an empty set.

    BOOST_FOREACH(Curve_segment_index curve_index, corner_tmp_incidences)
    {
      get_incidences(curve_index,
                     std::inserter(incidences,
                                   incidences.begin()));
    }

    //increment the loop variable
    ++cit;
  }
}

template <class MD_>
template <typename IndicesOutputIterator>
IndicesOutputIterator
cgalMeshDomainWithRationalBezierFeatures<MD_>::
get_incidences(Curve_segment_index id,
               IndicesOutputIterator indices_out) const
{
    typename Edges_incidences::const_iterator it = edges_incidences_.find(id);

    if(it == edges_incidences_.end()) return indices_out;

    const Surface_patch_index_set& incidences = it->second;

    return std::copy(incidences.begin(), incidences.end(), indices_out);
}

template <class MD_>
template <typename IndicesOutputIterator>
IndicesOutputIterator
cgalMeshDomainWithRationalBezierFeatures<MD_>::
get_corner_incidences(Corner_index id,
                      IndicesOutputIterator indices_out) const
{
    typename Corners_incidences::const_iterator it = corners_incidences_.find(id);
    const Surface_patch_index_set& incidences = it->second;
    return std::copy(incidences.begin(), incidences.end(), indices_out);
}

template <class MD_>
template <typename IndicesOutputIterator>
IndicesOutputIterator
cgalMeshDomainWithRationalBezierFeatures<MD_>::
get_corner_incident_curves(Corner_index id,
                           IndicesOutputIterator indices_out) const
{
    typename Corners_tmp_incidences::const_iterator it =
        corners_tmp_incidences_.find(id);
    const std::set<Curve_segment_index>& incidences = it->second;
    return std::copy(incidences.begin(), incidences.end(), indices_out);
}

template <class MD_>
const typename cgalMeshDomainWithRationalBezierFeatures<MD_>::Surface_patch_index_set&
cgalMeshDomainWithRationalBezierFeatures<MD_>::
get_incidences(Curve_segment_index id) const
{
    typename Edges_incidences::const_iterator it = edges_incidences_.find(id);
    return it->second;
}

template <class MD_>
typename cgalMeshDomainWithRationalBezierFeatures<MD_>::Point_3
cgalMeshDomainWithRationalBezierFeatures<MD_>::corner_point_from_index(Corner_index p_corner_index) const
{
    return corners_reverse_.at(p_corner_index);
}

template <class MD_>
typename cgalMeshDomainWithRationalBezierFeatures<MD_>::Corner_index
cgalMeshDomainWithRationalBezierFeatures<MD_>::
register_corner(const Point_3& p, const Curve_segment_index& curve_index)
{
    typename Corners::iterator cit = corners_.lower_bound(p);

    //If the corner already exists, returns...
    if(cit != corners_.end() && !(corners_.key_comp()(p, cit->first))) {
        corners_tmp_incidences_[cit->second].insert(curve_index);
        return cit->second;
    }

    //...else insert it!

    const Corner_index index = current_corner_index_;
    ++current_corner_index_;

    corners_.insert(cit, std::make_pair(p, index));
    corners_reverse_.insert(std::make_pair(index, p));
    corners_tmp_incidences_[index].insert(curve_index);
    return index;
}

template <class MD_>
CGAL::Sign
cgalMeshDomainWithRationalBezierFeatures<MD_>::
distance_sign(double p, double q, const Curve_segment_index& index) const
{
    typename Edges::const_iterator eit = edges_.find(index);
    CGAL_assertion(eit != edges_.end());

    CGAL_precondition( ! this->is_cycle(index));

    if ( p == q )
        return CGAL::ZERO;
    else if ( p < q )
        return CGAL::POSITIVE;
    else
        return CGAL::NEGATIVE;
}

template <class MD_>
CGAL::Sign
cgalMeshDomainWithRationalBezierFeatures<MD_>::
distance_sign_along_cycle(double p, double q, double r, const Curve_segment_index& index) const
{
    //Find edge
    typename Edges::const_iterator eit = edges_.find(index);
    CGAL_assertion(eit != edges_.end());

    //If eit is not a cycle, then the orientation corresponds to the sign
    //of the distance
    //SHOULDNT THERE BE A PROBLEM IF THERE IS NO CYCLE SINCE WE ASK FOR THE DISTANCE SIGN ALONG CYCLE ?
    if ( !this->is_cycle(index) )
        {
            return distance_sign(p, r, index);
        }

    //If p and r are the same point, it correspond to a complete loop on a cycle
    if ( p == r ) { return CGAL::POSITIVE; }

    // We are on a cycle without any clue (p==q). Return the shortest path as
    //
    //orientation.
    //THIS CANT HAPPEN
    //HOW TO DO THAT ? ...
    if ( p == q )
        {
            // FT pr = eit->second.geodesic_distance(p,r);
            // FT rp = eit->second.geodesic_distance(r,p);
            // if ( pr < rp ) { return CGAL::POSITIVE; }
            // else {
            return CGAL::NEGATIVE; // }
        }

    //If pq or pr is negative, edge is not a cycle, thus geodesic_distance
    //gives the answer.

    ///////////////////////////////////////////////////////////////////
    //TODO
    ///////////////////////////////////////////////////////////////////

    //Compare pq and pr
    // if ( pq <= pr ) { return CGAL::POSITIVE; }
    // else {
        return CGAL::NEGATIVE; // }
}

template <class MD_>
bool
cgalMeshDomainWithRationalBezierFeatures<MD_>::
is_cycle(const Point_3&, const Curve_segment_index& index) const
{
  return is_cycle(index);
}

template <class MD_>
bool
cgalMeshDomainWithRationalBezierFeatures<MD_>::
is_cycle(const Curve_segment_index& index) const
{
    //Find edge
    typename Edges::const_iterator eit = edges_.find(index);
    CGAL_assertion(eit != edges_.end());
    dtkContinuousGeometryPrimitives::Point_3 first_point(0., 0., 0.);
    dtkContinuousGeometryPrimitives::Point_3 last_point(0., 0., 0.);
    ///////////////////////////////////////////////////////////////////
    //Checks that the last control point is not equal to the first one
    ///////////////////////////////////////////////////////////////////
    eit->second.first->controlPoint(0, first_point.data());
    eit->second.first->controlPoint(eit->second.first->degree(), last_point.data());
    return (first_point == last_point);
}
} //namespace CGAL
