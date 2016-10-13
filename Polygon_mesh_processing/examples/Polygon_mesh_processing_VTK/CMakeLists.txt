cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

if (NOT ParaView_BINARY_DIR)
  find_package(ParaView QUIET)
  include(${PARAVIEW_USE_FILE})
endif()
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()
find_package(VTK QUIET)

find_package(CGAL)
include( ${CGAL_USE_FILE} )

include(ParaViewPlugins)
include(${VTK_USE_FILE})
include_directories(
  ${VTK_INCLUDE_DIRS})
if(ParaView_FOUND)
    if (PARAVIEW_BUILD_QT_GUI)
      if (PARAVIEW_QT_VERSION VERSION_GREATER "4")
        qt5_wrap_cpp(moc_srcs
          pqIsotropicRemeshingWidget.h)
      else ()
        qt4_wrap_cpp(moc_srcs
          pqIsotropicRemeshingWidget.h)
      endif ()
      set (outifaces0)
      set (outsrcs0)
      add_paraview_property_widget(outifaces0 outsrcs0
         TYPE "bbox_panel"
         CLASS_NAME pqIsotropicRemeshingWidget)

  ADD_PARAVIEW_PLUGIN(IsotropicRemeshingWithEdgesFilter "1.0"
    SERVER_MANAGER_XML IsotropicRemeshingWithEdgesFilter.xml
    SERVER_MANAGER_SOURCES vtkIsotropicRemeshingWithEdgesFilter.cxx)

  ADD_PARAVIEW_PLUGIN(IsotropicRemeshingFilter "1.0"
    SERVER_MANAGER_XML IsotropicRemeshingFilter.xml
    SERVER_MANAGER_SOURCES vtkIsotropicRemeshingFilter.cxx
    GUI_INTERFACES ${outifaces0}
    SOURCES ${outsrcs0}
            ${moc_srcs}
    pqIsotropicRemeshingWidget.cxx)
  endif(PARAVIEW_BUILD_QT_GUI)
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
    target_link_libraries(TestFilterEdges LINK_PRIVATE ${VTK_LIBRARIES})
    target_link_libraries(TestFilter LINK_PRIVATE ${VTK_LIBRARIES})
  endif()
endif()

if(CGAL_FOUND )
  # Link with CGAL
  if(ParaView_FOUND)
    target_link_libraries( IsotropicRemeshingWithEdgesFilter LINK_PRIVATE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
    target_link_libraries( IsotropicRemeshingFilter LINK_PRIVATE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
  endif(ParaView_FOUND)
  if(VTK_FOUND)
    target_link_libraries( TestFilterEdges LINK_PRIVATE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
    target_link_libraries( TestFilter LINK_PRIVATE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
  endif()
endif (CGAL_FOUND)

