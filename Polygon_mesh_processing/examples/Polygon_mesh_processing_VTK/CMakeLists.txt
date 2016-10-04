cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

if (NOT ParaView_BINARY_DIR)
  find_package(ParaView QUIET)
  include(${PARAVIEW_USE_FILE})
endif()
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()
find_package(VTK QUIET)


include(ParaViewPlugins)
include(${VTK_USE_FILE})
include_directories(
  ${VTK_INCLUDE_DIRS})
if(ParaView_FOUND)
  ADD_PARAVIEW_PLUGIN(IsotropicRemeshingWithEdgesFilter "1.0"
    SERVER_MANAGER_XML IsotropicRemeshingWithEdgesFilter.xml
    SERVER_MANAGER_SOURCES vtkIsotropicRemeshingWithEdgesFilter.cxx)

  ADD_PARAVIEW_PLUGIN(IsotropicRemeshingFilter "1.0"
    SERVER_MANAGER_XML IsotropicRemeshingFilter.xml
    SERVER_MANAGER_SOURCES vtkIsotropicRemeshingFilter.cxx)
else()
  message("WARNING : The Paraview plugins need Paraview, so they won't be compiled.")
endif(ParaView_FOUND)

if(VTK_FOUND)
  add_executable(TestFilterEdges MACOSX_BUNDLE test_isotropic_remeshing_with_edges.cpp vtkIsotropicRemeshingWithEdgesFilter.cxx)
  add_executable(TestFilter MACOSX_BUNDLE test_isotropic_remeshing.cpp vtkIsotropicRemeshingFilter.cxx)
else()
  message("WARNING : The VTK filter tests need VTK, so they won't be compiled.")
endif()

if(VTK_FOUND)
  if( VTK_LIBRARIES)
      message("LIB FOUND: ${VTK_LIBRARIES}")
    target_link_libraries(TestFilterEdges ${VTK_LIBRARIES})
    target_link_libraries(TestFilter ${VTK_LIBRARIES})
  endif()
endif()

find_package(CGAL)
include( ${CGAL_USE_FILE} )

if(CGAL_FOUND )
  # Link with CGAL
  if(ParaView_FOUND)
    target_link_libraries( IsotropicRemeshingWithEdgesFilter LINK_PRIVATE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
    target_link_libraries( IsotropicRemeshingFilter LINK_PRIVATE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
  endif(ParaView_FOUND)
  if(VTK_FOUND)
    target_link_libraries( TestFilterEdges ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
    target_link_libraries( TestFilter ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
  endif()
endif (CGAL_FOUND)

