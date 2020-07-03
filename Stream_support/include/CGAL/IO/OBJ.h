// Copyright (c) 2015  Geometry Factory
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL$
// $Id$
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s) : Lutz Kettner
//             Andreas Fabri
//             Maxime Gimeno

#ifndef CGAL_IO_OBJ_H
#define CGAL_IO_OBJ_H

#include <CGAL/IO/OBJ/File_writer_wavefront.h>
#include <CGAL/IO/Generic_writer.h>

#include <CGAL/Container_helper.h>
#include <CGAL/IO/io.h>
#include <boost/range/value_type.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace CGAL {

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Read

namespace IO {
namespace internal {

template <typename PointRange, typename PolygonRange, typename VertexNormalOutputIterator, typename VertexTextureOutputIterator>
bool read_OBJ(std::istream& is,
              PointRange& points,
              PolygonRange& faces,
              VertexNormalOutputIterator,
              VertexTextureOutputIterator,
              bool verbose = true)
{
  if(!is.good())
  {
    if(verbose)
      std::cerr<<"File doesn't exist."<<std::endl;
    return false;
  }

  typedef typename boost::range_value<PointRange>::type                               Point;

  set_ascii_mode(is); // obj is ASCII only

  int mini(1), maxi(-1);
  bool first_o = true;
  std::string s;
  Point p;

  std::string line;
  bool tex_found(false), norm_found(false);
  std::size_t offset_idx=0;
  while(getline(is, line))
  {
    if(line.empty())
      continue;

    std::istringstream iss(line);
    if(!(iss >> s))
      continue; // can't read anything on the line, whitespace only?
    if(s == "o")
    {
      if(!first_o)
      {
        if(maxi == static_cast<int>(offset_idx -1 ) && mini == static_cast<int>(offset_idx + 1))
        {
          if(verbose)
            std::cerr << "No face detected." << std::endl;
          return false;
        }

        if(maxi > static_cast<int>(points.size()-offset_idx) || mini < -static_cast<int>(points.size()-offset_idx))
        {
          if(verbose)
            std::cerr << "a face index is invalid " << std::endl;
          return false;
        }
        mini = 1;
        maxi = -1;
      }
      else
      {
        first_o = false;
      }
      offset_idx = points.size();
    }

    if(s == "v")
    {
      if(!(iss >> p))
      {
        if(verbose)
          std::cerr << "error while reading OBJ vertex." << std::endl;
        return false;
      }

      points.push_back(p);
    }
    else if(s == "vt")
    {
      tex_found = true;
    }
    else if(s == "vn")
    {
      norm_found = true;
    }
    else if(s == "f")
    {
      int i;
      faces.emplace_back();
      while(iss >> i)
      {
        if(i < 1)
        {
          const std::size_t n = faces.back().size();
          ::CGAL::internal::resize(faces.back(), n + 1);
          faces.back()[n] = points.size() + offset_idx + i; // negative indices are relative references
          if(i < mini)
            mini = i;
        }
        else
        {
          const std::size_t n = faces.back().size();
          ::CGAL::internal::resize(faces.back(), n + 1);
          faces.back()[n] = i + offset_idx - 1;
          if(i-1 > maxi)
            maxi = i-1;
        }

        // the format can be "f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ..." and we only read vertex ids for now,
        // so skip to the next vertex
        iss.ignore(256, ' ');
      }

      if(iss.bad())
        return false;
    }
    else
    {
      // std::cerr << "ERROR : Cannnot read line beginning with " << line[0] << std::endl;
     continue;
    }
  }
  if(norm_found && verbose)
    std::cout<<"WARNING: normals were found in this file, but were discarded."<<std::endl;
  if(tex_found && verbose)
    std::cout<<"WARNING: textures were found in this file, but were discarded."<<std::endl;
  if(maxi == static_cast<int>(offset_idx - 1) && mini == static_cast<int>(offset_idx + 1))
  {
    if(verbose)
      std::cerr << "No face detected." << std::endl;
    return false;
  }

  if(maxi > static_cast<int>(points.size()-offset_idx) || mini < -static_cast<int>(points.size()-offset_idx))
  {
    if(verbose)
      std::cerr << "a face index is invalid " << std::endl;
    return false;
  }

  return !is.bad();
}

} // namespace internal
} // namespace IO

/// \cond SKIP_IN_MANUAL

template <typename PointRange, typename PolygonRange, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool read_OBJ(std::istream& is,
              PointRange& points,
              PolygonRange& faces,
              const CGAL_BGL_NP_CLASS&,
              bool verbose = true)
{
  return IO::internal::read_OBJ(is, points, faces,
                                CGAL::Emptyset_iterator(), CGAL::Emptyset_iterator(),
                                verbose);
}

template <typename PointRange, typename PolygonRange, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool read_OBJ(const char* fname, PointRange& points, PolygonRange& polygons,
              const CGAL_BGL_NP_CLASS& np, bool verbose = true)
{
  std::ifstream in(fname);
  return read_OBJ(in, points, polygons, np, verbose);
}

template <typename PointRange, typename PolygonRange, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool read_OBJ(const std::string& fname, PointRange& points, PolygonRange& polygons,
              const CGAL_BGL_NP_CLASS& np, bool verbose = true)
{
  return read_OBJ(fname.c_str(), points, polygons, np, verbose);
}

/// \endcond

/// \ingroup PkgStreamSupportIoFuncsOBJ
///
/// \brief reads the content of `is` into `points` and `polygons`, using the \ref IOStreamOBJ.
///
/// \tparam PointRange a model of the concept `RandomAccessContainer` whose value type is the point type.
/// \tparam PolygonRange a model of the concept `SequenceContainer`
///                      whose value_type is itself a model of the concept `SequenceContainer`
///                      whose value_type is an integer type.
///
/// \param is the input stream
/// \param points points of the soup of polygons.
/// \param polygons a `PolygonRange`. Each element in it describes a polygon
///        using the indices of the points in `points`.
///
/// \returns `true` if the reading was successful, `false` otherwise.
template <typename PointRange, typename PolygonRange>
bool read_OBJ(std::istream& is, PointRange& points, PolygonRange& polygons)
{
  return read_OBJ(is, points, polygons, parameters::all_default());
}

/// \ingroup PkgStreamSupportIoFuncsOBJ
///
/// \brief reads the content of the file `fname` into `points` and `polygons`, using the \ref IOStreamOBJ.
///
/// \tparam PointRange a model of the concept `RandomAccessContainer` whose value type is the point type.
/// \tparam PolygonRange a model of the concept `SequenceContainer`
///                      whose value_type is itself a model of the concept `SequenceContainer`
///                      whose value_type is an integer type.
///
/// \param fname the path to the input file
/// \param points points of the soup of polygons.
/// \param polygons a `PolygonRange`. Each element in it describes a polygon
///        using the indices of the points in `points`.
///
/// \returns `true` if the reading was successful, `false` otherwise.
template <typename PointRange, typename PolygonRange>
bool read_OBJ(const char* fname, PointRange& points, PolygonRange& polygons)
{
  return read_OBJ(fname, points, polygons, parameters::all_default());
}

/// \cond SKIP_IN_MANUAL

template <typename PointRange, typename PolygonRange>
bool read_OBJ(const std::string& fname, PointRange& points, PolygonRange& polygons)
{
  return read_OBJ(fname, points, polygons, parameters::all_default());
}

/// \endcond

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Write

/*!
 * \ingroup PkgStreamSupportIoFuncsOBJ
 *
 * \brief writes the content of `points` and `polygons` in `os`, using the \ref IOStreamOBJ.
 *
 * \tparam PointRange a model of the concept `RandomAccessContainer` whose value type is the point type.
 * \tparam PolygonRange a model of the concept `SequenceContainer`
 *                      whose value_type is itself a model of the concept `SequenceContainer`
 *                      whose value_type is an integer type.
 * \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
 *
 * \param os the output stream
 * \param points points of the soup of polygons.
 * \param polygons a `PolygonRange`. Each element in it describes a polygon
 *        using the indices of the points in `points`.
 * \param np optional sequence of \ref bgl_namedparameters "Named Parameters" among the ones listed below
 *
 * \cgalNamedParamsBegin
 *   \cgalParamNBegin{stream_precision}
 *     \cgalParamDescription{a parameter used to set the precision (i.e. how many digits are generated) of the output stream}
 *     \cgalParamType{int}
 *     \cgalParamDefault{`6`}
 *   \cgalParamNEnd
 * \cgalNamedParamsEnd
 *
 * \return `true` if the writing was successful, `false` otherwise.
 */
template <typename PointRange,
          typename PolygonRange,
          typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool write_OBJ(std::ostream& os,
               const PointRange& points,
               const PolygonRange& polygons,
               const CGAL_BGL_NP_CLASS& np)
{
  set_ascii_mode(os); // obj is ASCII only
  Generic_writer<std::ostream, File_writer_wavefront> writer(os);
  return writer(points, polygons, np);
}

/// \cond SKIP_IN_MANUAL

template <typename PointRange, typename PolygonRange>
bool write_OBJ(std::ostream& os, const PointRange& points, const PolygonRange& polygons)
{
  return write_OBJ(os, points, polygons, parameters::all_default());
}

/// \endcond

/*!
 * \ingroup PkgStreamSupportIoFuncsOBJ
 *
 * \brief writes the content of `points` and `polygons` in a file named `fname`, using the \ref IOStreamOBJ.
 *
 * \tparam PointRange a model of the concept `RandomAccessContainer` whose value type is the point type.
 * \tparam PolygonRange a model of the concept `SequenceContainer`
 *                      whose value_type is itself a model of the concept `SequenceContainer`
 *                      whose value_type is an integer type.
 * \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
 *
 * \param fname the path to the output file
 * \param points points of the soup of polygons.
 * \param polygons a `PolygonRange`. Each element in it describes a polygon
 *        using the indices of the points in `points`.
 * \param np optional sequence of \ref bgl_namedparameters "Named Parameters" among the ones listed below
 *
 * \cgalNamedParamsBegin
 *   \cgalParamNBegin{stream_precision}
 *     \cgalParamDescription{a parameter used to set the precision (i.e. how many digits are generated) of the output stream}
 *     \cgalParamType{int}
 *     \cgalParamDefault{`6`}
 *   \cgalParamNEnd
 * \cgalNamedParamsEnd
 *
 * \return `true` if the writing was successful, `false` otherwise.
 */
template <typename PointRange,
          typename PolygonRange,
          typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool write_OBJ(const char* fname, const PointRange& points, const PolygonRange& polygons,
               const CGAL_BGL_NP_CLASS& np)
{
  std::ofstream out(fname);
  return write_OBJ(out, points, polygons, np);
}

/// \cond SKIP_IN_MANUAL

template <typename PointRange, typename PolygonRange>
bool write_OBJ(const char* fname, const PointRange& points, const PolygonRange& polygons)
{
  return write_OBJ(fname, points, polygons, parameters::all_default());
}

template <typename PointRange, typename PolygonRange, typename CGAL_BGL_NP_TEMPLATE_PARAMETERS>
bool write_OBJ(const std::string& fname, const PointRange& points, const PolygonRange& polygons, const CGAL_BGL_NP_CLASS& np)
{
  return write_OBJ(fname.c_str(), points, polygons, np);
}

template <typename PointRange, typename PolygonRange>
bool write_OBJ(const std::string& fname, const PointRange& points, const PolygonRange& polygons)
{
  return write_OBJ(fname.c_str(), points, polygons, parameters::all_default());
}

/// \endcond

} // namespace CGAL

#endif // CGAL_IO_OBJ_H
