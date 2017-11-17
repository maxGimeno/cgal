set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(dtk 1.0.0 REQUIRED)
find_package(dtkContinuousGeometry 0.0.1 REQUIRED)
find_package(cgalCaoProtection 0.0.1 REQUIRED)
find_package(dtkNurbsProbing 0.0.1 REQUIRED)
find_package(Blaze REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(${dtk_INCLUDE_DIRS})
include_directories(${dtkContinuousGeometry_INCLUDE_DIRS})

include_directories(${cgalCaoProtection_INCLUDE_DIRS})
include_directories(${dtkNurbsProbing_INCLUDE_DIRS})

include_directories(${Blaze_INCLUDE_DIR})

include( polyhedron_demo_macros )

qt5_wrap_ui(CAD_initialization_plugin_UI_FILES CAD_mesher_protect_initialization_dialog.ui)
polyhedron_demo_plugin(cAD_initialization_plugin
  CAD_initialization_plugin
  ${CAD_initialization_plugin_UI_FILES})
target_link_libraries(cAD_initialization_plugin
  polyhedron_demo
  scene_cad_item
  scene_c3t3_item
  scene_c3t3_cad_item
  dtkCore
  dtkContinuousGeometryCore
  dtkNurbsProbingCore)

qt5_wrap_ui(CAD_refine_plugin_UI_FILES CAD_refine_dialog.ui)
polyhedron_demo_plugin(cAD_refine_plugin
  CAD_refine_plugin
  ${CAD_refine_plugin_UI_FILES})

target_link_libraries(cAD_refine_plugin
  polyhedron_demo
  scene_cad_item
  scene_c3t3_item
  scene_c3t3_cad_item
  dtkCore
  dtkContinuousGeometryCore
  dtkNurbsProbingCore)

polyhedron_demo_plugin(cad_meshing_plugin
  Cad_meshing_plugin)
target_link_libraries(cad_meshing_plugin
  polyhedron_demo
  scene_cad_item
  dtkCore
  dtkContinuousGeometryCore)

polyhedron_demo_plugin(rational_bezier_surface_plugin
  Rational_bezier_surface_plugin)
target_link_libraries(rational_bezier_surface_plugin
  polyhedron_demo
  scene_rational_bezier_surface_item
  dtkCore
  dtkContinuousGeometryCore)

add_item(scene_c3t3_cad_item Scene_c3t3_cad_item.cpp)
target_link_libraries(scene_c3t3_cad_item
  scene_polyhedron_item
  scene_polygon_soup_item
  scene_basic_objects
  ${TBB_LIBRARIES})

qt5_wrap_ui( cad_mesherUI_FILES Cad_mesher_dialog.ui)
polyhedron_demo_plugin(cad_mesher_plugin
  Cad_mesher_plugin
  ${cad_mesherUI_FILES})
target_link_libraries(cad_mesher_plugin
  polyhedron_demo
  scene_c3t3_item
  scene_cad_item
  scene_c3t3_cad_item
  dtkCore
  dtkContinuousGeometryCore
  dtkNurbsProbingCore)
