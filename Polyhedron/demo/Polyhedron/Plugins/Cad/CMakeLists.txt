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
polyhedron_demo_plugin(cad_meshing_plugin Cad_meshing_plugin)
target_link_libraries(cad_meshing_plugin scene_cad_item dtkCore dtkContinuousGeometryCore polyhedron_demo)

add_item(scene_c3t3_cad_item Scene_c3t3_cad_item.cpp)
target_link_libraries(scene_c3t3_cad_item scene_polyhedron_item scene_polygon_soup_item scene_basic_objects ${TBB_LIBRARIES})

qt5_wrap_ui( cad_mesherUI_FILES Cad_mesher_dialog.ui)
polyhedron_demo_plugin(cad_mesher_plugin Cad_mesher_plugin ${cad_mesherUI_FILES})
target_link_libraries(cad_mesher_plugin scene_cad_item scene_c3t3_cad_item dtkCore dtkContinuousGeometryCore dtkNurbsProbingCore polyhedron_demo scene_c3t3_item)
