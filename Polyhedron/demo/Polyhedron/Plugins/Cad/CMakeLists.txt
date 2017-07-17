find_package(dtk 1.0.0 REQUIRED)
find_package(dtkContinuousGeometry 0.0.1 REQUIRED)

include_directories(${dtk_INCLUDE_DIRS})
include_directories(${dtkContinuousGeometry_INCLUDE_DIRS})

include( polyhedron_demo_macros )
polyhedron_demo_plugin(cad_meshing_plugin Cad_meshing_plugin)
target_link_libraries(cad_meshing_plugin scene_cad_item dtkCore dtkContinuousGeometryCore polyhedron_demo)

qt5_wrap_ui( cad_mesherUI_FILES Cad_mesher_dialog.ui)
polyhedron_demo_plugin(cad_mesher_plugin Cad_mesher_plugin ${cad_mesherUI_FILES})
target_link_libraries(cad_mesher_plugin scene_cad_item dtkCore dtkContinuousGeometryCore polyhedron_demo scene_c3t3_item)
