// Copyright (c) 2015  GeometryFactory (France).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL$
// $Id$
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Andreas Fabri
//                 Mael Rouxel-Labbé

#ifndef CGAL_BGL_IO_STL_H
#define CGAL_BGL_IO_STL_H

#include <CGAL/boost/graph/IO/Generic_facegraph_builder.h>
#include <CGAL/IO/STL.h>

#include <CGAL/assertions.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/named_params_helper.h>

#include <fstream>
#include <iostream>
#include <string>

#ifdef DOXYGEN_RUNNING
#define CGAL_BGL_NP_TEMPLATE_PARAMETERS NamedParameters
#define CGAL_BGL_NP_CLASS NamedParameters
#endif

namespace CGAL {

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Read

namespace IO {
namespace internal {

// Use CRTP to gain access to the protected members without getters/setters.
template <typename Graph, typename Point>
class STL_builder
  : public Generic_facegraph_builder<Graph, Point, STL_builder<Graph, Point> >
{
  typedef STL_builder<Graph, Point>                                             Self;
  typedef Generic_facegraph_builder<Graph, Point, Self>                         Base;

  typedef typename Base::Point_container                                        Point_container;
  typedef typename Base::Face                                                   Face;
  typedef typename Base::Face_container                                         Face_container;

public:
  STL_builder(std::istream& is, bool verbose) : Base(is, verbose) { }

  template <typename NamedParameters>
  bool read(std::istream& is,
            Point_container& points,
            Face_container& faces,
            const NamedParameters& np,
            bool verbose)
  {
    return read_STL(is, points, faces, np, verbose);
  }
};

} // namespace internal
} // namespace IO

/*!
  \ingroup PkgBGLIoFuncsSTL

  \brief reads the graph `g` from the input stream, using the \ref IOStreamSTL.

  \attention The graph `g` is not cleared, and the data from the stream is added.

  \tparam Graph a model of `MutableFaceGraph`
  \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"

  \param is the input stream
  \param g the graph to be built from the input data
  \param verbose whether extra information is printed when an incident occurs during reading
  \param np optional \ref bgl_namedparameters "Named Parameters" described below

  \cgalNamedParamsBegin
    \cgalParamNBegin{vertex_point_map}
      \cgalParamDescription{a property map associating points to the vertices of `g`}
      \cgalParamType{a class model of `ReadWritePropertyMap` with `boost::graph_traits<Graph>::%vertex_descriptor`
                     as key type and `%Point_3` as value type}
      \cgalParamDefault{`boost::get(CGAL::vertex_point, g)`}
      \cgalParamExtra{If this parameter is omitted, an internal property map for `CGAL::vertex_point_t`
                      must be available in `Graph`.}
    \cgalParamNEnd
  \cgalNamedParamsEnd

  \pre The data must represent a 2-manifold

  \returns `true` if the resulting mesh is valid.

  \sa Overloads of this function for specific models of the concept `FaceGraph`.
*/
template <typename Graph, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool read_STL(std::istream& is,
              Graph& g,
              const CGAL_BGL_NP_CLASS& np,
              bool verbose = true)
{
  typedef typename CGAL::GetVertexPointMap<Graph, CGAL_BGL_NP_CLASS>::type      VPM;
  typedef typename boost::property_traits<VPM>::value_type                      Point;

  IO::internal::STL_builder<Graph, Point> builder(is, verbose);
  return builder(g, np);
}

/*!
  \ingroup PkgBGLIoFuncsSTL

  \brief reads the graph `g` from the file `fname`, using the \ref IOStreamSTL.

  \attention The graph `g` is not cleared, and the data from the stream is added.

  \tparam Graph a model of `MutableFaceGraph`
  \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"

  \param fname the name of the input file
  \param g the graph to be built from the input data
  \param verbose whether extra information is printed when an incident occurs during reading
  \param np optional \ref bgl_namedparameters "Named Parameters" described below

  \cgalNamedParamsBegin
    \cgalParamNBegin{vertex_point_map}
      \cgalParamDescription{a property map associating points to the vertices of `g`}
      \cgalParamType{a class model of `ReadWritePropertyMap` with `boost::graph_traits<Graph>::%vertex_descriptor`
                     as key type and `%Point_3` as value type}
      \cgalParamDefault{`boost::get(CGAL::vertex_point, g)`}
      \cgalParamExtra{If this parameter is omitted, an internal property map for `CGAL::vertex_point_t`
                      must be available in `Graph`.}
    \cgalParamNEnd
  \cgalNamedParamsEnd

  \pre The data must represent a 2-manifold

  \returns `true` if the resulting mesh is valid.

  \sa Overloads of this function for specific models of the concept `FaceGraph`.
*/
template <typename Graph, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool read_STL(const char* fname, Graph& g, const CGAL_BGL_NP_CLASS& np,
              bool verbose = true)
{
  std::ifstream is(fname);
  return read_STL(is, g, np, verbose);
}

/// \cond SKIP_IN_MANUAL

template <typename Graph, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool read_STL(const std::string& fname, Graph& g, const CGAL_BGL_NP_CLASS& np,
              bool verbose = true)
{
  return read_STL(fname.c_str(), g, np, verbose);
}

template <typename Graph>
bool read_STL(std::istream& is, Graph& g) { return read_STL(is, g, parameters::all_default()); }
template <typename Graph>
bool read_STL(const char* fname, Graph& g) { return read_STL(fname, g, parameters::all_default()); }
template <typename Graph>
bool read_STL(const std::string& fname, Graph& g) { return read_STL(fname, g, parameters::all_default()); }

/// \endcond

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Write

/*!
  \ingroup PkgBGLIoFuncsSTL

  \brief writes the graph `g` in the output stream `os`, using the \ref IOStreamSTL.

  \tparam Graph a model of `FaceListGraph` and `HalfedgeListGraph`
  \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"

  \param os the output stream
  \param g the graph to be output
  \param np optional \ref bgl_namedparameters "Named Parameters" described below

  \cgalNamedParamsBegin
    \cgalParamNBegin{vertex_point_map}
      \cgalParamDescription{a property map associating points to the vertices of `g`}
      \cgalParamType{a class model of `ReadablePropertyMap` with `boost::graph_traits<Graph>::%vertex_descriptor`
                     as key type and `%Point_3` as value type}
      \cgalParamDefault{`boost::get(CGAL::vertex_point, g)`}
      \cgalParamExtra{If this parameter is omitted, an internal property map for `CGAL::vertex_point_t`
                      must be available in `Graph`.}
    \cgalParamNEnd

    \cgalParamNBegin{stream_precision}
      \cgalParamDescription{a parameter used to set the precision (i.e. how many digits are generated) of the output stream}
       \cgalParamType{int}
       \cgalParamDefault{`6`}
    \cgalParamNEnd
  \cgalNamedParamsEnd

  \pre The graph must contain only triangle faces.

  \returns `true` if writing was successful.
*/
template <typename Graph, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool write_STL(std::ostream& os,
               const Graph& g,
               const CGAL_BGL_NP_CLASS& np)
{
  typedef typename boost::graph_traits<Graph>::halfedge_descriptor                  halfedge_descriptor;
  typedef typename boost::graph_traits<Graph>::face_descriptor                      face_descriptor;

  typedef typename CGAL::GetVertexPointMap<Graph, CGAL_BGL_NP_CLASS>::const_type    VPM;
  typedef typename boost::property_traits<VPM>::reference                           Point_ref;
  typedef typename boost::property_traits<VPM>::value_type                          Point;
  typedef typename Kernel_traits<Point>::Kernel::Vector_3                           Vector;

  using parameters::choose_parameter;
  using parameters::get_parameter;

  VPM vpm = choose_parameter(get_parameter(np, internal_np::vertex_point),
                             get_const_property_map(CGAL::vertex_point, g));

  if(!os.good())
    return false;

  const int precision = choose_parameter(get_parameter(np, internal_np::stream_precision), 6);
  os << std::setprecision(precision);

  if(get_mode(os) == IO::BINARY)
  {
    os << "FileType: Binary                                                                ";
    const boost::uint32_t N32 = static_cast<boost::uint32_t>(faces(g).size());
    os.write(reinterpret_cast<const char *>(&N32), sizeof(N32));

    for(const face_descriptor f : faces(g))
    {
      const halfedge_descriptor h = halfedge(f, g);
      Point_ref p = get(vpm, target(h, g));
      Point_ref q = get(vpm, target(next(h, g), g));
      Point_ref r = get(vpm, source(h, g));

      Vector n = collinear(p, q, r) ? Vector(1, 0, 0) : unit_normal(p, q, r);

      const float coords[12] =
      {
        static_cast<float>(to_double(n.x())), static_cast<float>(to_double(n.y())), static_cast<float>(to_double(n.z())),
        static_cast<float>(to_double(p.x())), static_cast<float>(to_double(p.y())), static_cast<float>(to_double(p.z())),
        static_cast<float>(to_double(q.x())), static_cast<float>(to_double(q.y())), static_cast<float>(to_double(q.z())),
        static_cast<float>(to_double(r.x())), static_cast<float>(to_double(r.y())), static_cast<float>(to_double(r.z())) };

      for(int i=0; i<12; ++i)
        os.write(reinterpret_cast<const char *>(&coords[i]), sizeof(coords[i]));
      os << "  ";
    }
  }
  else
  {
    os << "solid" << std::endl;
    for(const face_descriptor f : faces(g))
    {
      halfedge_descriptor h = halfedge(f, g);
      Point_ref p = get(vpm, target(h, g));
      Point_ref q = get(vpm, target(next(h, g), g));
      Point_ref r = get(vpm, source(h, g));
      Vector n = collinear(p, q, r) ? Vector(1, 0, 0) : unit_normal(p, q, r);

      os << "facet normal " << n << "\nouter loop"<< "\n";
      os << "vertex " << p << "\n";
      os << "vertex " << q << "\n";
      os << "vertex " << r << "\n";
      os << "endloop\nendfacet" << std::endl;
    }
    os << "endsolid" << std::endl;
  }

  return os.good();
}

/*!
\ingroup PkgBGLIoFuncsSTL

 \brief writes the graph `g` into a file named `fname`, using the \ref IOStreamSTL.

 \tparam Graph a model of `FaceListGraph` and `HalfedgeListGraph`
 \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"

 \param fname the name of the output stream
 \param g the graph to be output
 \param np optional \ref bgl_namedparameters "Named Parameters" described below

 \cgalNamedParamsBegin
    \cgalParamNBegin{use_binary_mode}
      \cgalParamDescription{indicates whether data should be written in binary (`true`) or in ASCII (`false`)}
      \cgalParamType{Boolean}
      \cgalParamDefault{`true`}
    \cgalParamNEnd

    \cgalParamNBegin{vertex_point_map}
      \cgalParamDescription{a property map associating points to the vertices of `g`}
      \cgalParamType{a class model of `ReadWritePropertyMap` with `boost::graph_traits<Graph>::%vertex_descriptor`
                     as key type and `%Point_3` as value type}
      \cgalParamDefault{`boost::get(CGAL::vertex_point, g)`}
      \cgalParamExtra{If this parameter is omitted, an internal property map for `CGAL::vertex_point_t`
                      must be available in `Graph`.}
    \cgalParamNEnd

    \cgalParamNBegin{stream_precision}
      \cgalParamDescription{a parameter used to set the precision (i.e. how many digits are generated) of the output stream}
       \cgalParamType{int}
       \cgalParamDefault{`6`}
    \cgalParamNEnd
 \cgalNamedParamsEnd

 \pre The graph must contain only triangle faces.

 \sa Overloads of this function for specific models of the concept `FaceGraph`.
*/
template <typename Graph, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool write_STL(const char* fname, const Graph& g, const CGAL_BGL_NP_CLASS& np)
{
  std::ofstream os(fname);
  const bool binary = CGAL::parameters::choose_parameter(CGAL::parameters::get_parameter(np, internal_np::use_binary_mode), true);
  if(binary)
    CGAL::set_mode(os, CGAL::IO::BINARY);

  return write_STL(os, g, np);
}

/// \cond SKIP_IN_MANUAL

template <typename Graph, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool write_STL(const std::string& fname, const Graph& g, const CGAL_BGL_NP_CLASS& np)
{
  return write_STL(fname.c_str(), g, np);
}

template <typename Graph>
bool write_STL(std::ostream& os, const Graph& g) { return write_STL(os, g, parameters::all_default()); }
template <typename Graph>
bool write_STL(const char* fname, const Graph& g) { return write_STL(fname, g, parameters::all_default()); }
template <typename Graph>
bool write_STL(const std::string& fname, const Graph& g) { return write_STL(fname, g, parameters::all_default()); }

/// \endcond

} // namespace CGAL

#endif // CGAL_BGL_IO_STL_H
