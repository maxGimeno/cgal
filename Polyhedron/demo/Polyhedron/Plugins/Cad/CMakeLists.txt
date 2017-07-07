find_package(dtk 1.0.0 REQUIRED)
find_package(dtkContinuousGeometry 0.0.1 REQUIRED)

include_directories(${dtk_INCLUDE_DIRS})
include_directories(${dtkContinuousGeometry_INCLUDE_DIRS})

include( polyhedron_demo_macros )
polyhedron_demo_plugin(cad_meshing_plugin Cad_meshing_plugin)
target_link_libraries(cad_meshing_plugin scene_cad_item dtkCore dtkContinuousGeometryCore polyhedron_demo)

