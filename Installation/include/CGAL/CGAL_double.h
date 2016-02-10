// Copyright (c) 1997-2013
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved. 
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
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

#ifndef CGAL_GL_DOUBLE_H
#define CGAL_GL_DOUBLE_H

//Defines type for double. Needed because OpenGL ES doesn't know double, only float.
#if ANDROID
  #define CGAL_GLdouble GLfloat
  #define CGAL_GL_DOUBLE GL_FLOAT
#else
  #define CGAL_GLdouble GLdouble
  #define CGAL_GL_DOUBLE GL_DOUBLE
#endif 

#endif //CGAL_GL_DOUBLE_H
