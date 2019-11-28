set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(dtk 1.0.0 REQUIRED)
find_package(dtkContinuousGeometry 0.0.1 REQUIRED)
find_package(dtkNurbsProbing 0.0.1 REQUIRED)
find_package(Blaze REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(${dtk_INCLUDE_DIRS})
include_directories(${dtkContinuousGeometry_INCLUDE_DIRS})

include_directories(${dtkNurbsProbing_INCLUDE_DIRS})

include_directories(${Blaze_INCLUDE_DIR})

include( polyhedron_demo_macros )

qt5_wrap_ui(CAD_initialization_plugin_UI_FILES CAD_mesher_protect_initialization_dialog.ui)
polyhedron_demo_plugin(cad_initialization_plugin
  CAD_initialization_plugin
  ${CAD_initialization_plugin_UI_FILES} KEYWORDS NURBS)
target_link_libraries(cad_initialization_plugin PUBLIC
  polyhedron_demo
  scene_cad_item
  scene_c3t3_item
  scene_c3t3_cad_item
  dtkCore
  dtkContinuousGeometryCore
  dtkNurbsProbingCore)

qt5_wrap_ui(CAD_refine_plugin_UI_FILES CAD_refine_dialog.ui)
polyhedron_demo_plugin(cad_refine_plugin
  CAD_refine_plugin
  ${CAD_refine_plugin_UI_FILES} KEYWORDS NURBS)
target_link_libraries(cad_refine_plugin PUBLIC
  polyhedron_demo
  scene_cad_item
  scene_c3t3_item
  scene_c3t3_cad_item
  dtkCore
  dtkContinuousGeometryCore
  dtkNurbsProbingCore)

polyhedron_demo_plugin(cad_meshing_plugin
  Cad_meshing_plugin KEYWORDS NURBS)
target_link_libraries(cad_meshing_plugin PUBLIC
  polyhedron_demo
  scene_cad_item
  dtkCore
  dtkContinuousGeometryCore)

polyhedron_demo_plugin(rational_bezier_surface_plugin
  Rational_bezier_surface_plugin KEYWORDS NURBS)
target_link_libraries(rational_bezier_surface_plugin PUBLIC
  polyhedron_demo
  scene_rational_bezier_surface_item
  dtkCore
  dtkContinuousGeometryCore)

polyhedron_demo_plugin(rational_bezier_curve_plugin
  Rational_bezier_curve_plugin)
target_link_libraries(rational_bezier_curve_plugin PUBLIC
  polyhedron_demo
  scene_rational_bezier_curve_item
  dtkCore
  dtkContinuousGeometryCore)

polyhedron_demo_plugin(nurbs_surface_plugin
  Nurbs_surface_plugin KEYWORDS NURBS)
target_link_libraries(nurbs_surface_plugin PUBLIC
  polyhedron_demo
  scene_nurbs_item
  dtkCore
  dtkContinuousGeometryCore)

add_item(scene_c3t3_cad_item Scene_c3t3_cad_item.cpp)
target_link_libraries(scene_c3t3_cad_item PUBLIC
  scene_polygon_soup_item
  scene_basic_objects
  ${TBB_LIBRARIES})

qt5_wrap_ui( cad_mesherUI_FILES Cad_mesher_dialog.ui)
polyhedron_demo_plugin(cad_mesher_plugin
  Cad_mesher_plugin
  ${cad_mesherUI_FILES} KEYWORDS NURBS)
target_link_libraries(cad_mesher_plugin PUBLIC
  polyhedron_demo
  scene_c3t3_item
  scene_cad_item
  scene_c3t3_cad_item
  dtkCore
  dtkContinuousGeometryCore
  dtkNurbsProbingCore)

polyhedron_demo_plugin(cad_c3t3_io_plugin Cad_c3t3_io_plugin KEYWORDS NURBS)
target_link_libraries(cad_c3t3_io_plugin PUBLIC scene_c3t3_cad_item)
