// Copyright (c) 2017  GeometryFactory Sarl (France)
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
// Author(s)     : Maxime Gimeno
#ifndef TRIANGLE_CONTAINER_H
#define TRIANGLE_CONTAINER_H

#include <CGAL/license/Three.h>


#include <CGAL/Three/Scene_item.h>
#include <CGAL/Three/Scene_interface.h>
#include <CGAL/Three/Viewer_interface.h>

using namespace CGAL::Three;
#ifdef demo_framework_EXPORTS
#  define DEMO_FRAMEWORK_EXPORT Q_DECL_EXPORT
#else
#  define DEMO_FRAMEWORK_EXPORT Q_DECL_IMPORT
#endif
namespace CGAL {
namespace Three {

struct DEMO_FRAMEWORK_EXPORT Triangle_container
{
  enum vbosName {
    Flat_vertices = 0,
    Smooth_vertices,
    Vertex_indices,
    Flat_normals,
    Smooth_normals,
    Facet_barycenters,
    VColors,
    FColors,
    NbOfVbos
  };

    Triangle_container(Scene_item *item, CGAL::Three::Viewer_interface *viewer, int program, bool indexed);

    void draw(const Scene_item &item, CGAL::Three::Viewer_interface* viewer) const;
    void initializeBuffers() const;
    void reset_vbos();
    mutable Vao* VAO;
    mutable std::vector<Vbo*> VBOs;
    mutable std::size_t nb_flat;
    mutable std::size_t idx_size;
    int program_id;
    mutable bool is_init;
    bool indexed;

    //drawing variables
    QVector4D plane;
    float shrink_factor;
    QColor color;
}; //end of class Triangle_container

}
}

#endif // TRIANGLE_CONTAINER_H
