find_program( GCOV_PATH gcov )
if(NOT GCOV_PATH)
  message(FATAL_ERROR "gcov not found! Aborting...")
endif(NOT GCOV_PATH)
find_package(PythonInterp 2.6.7)
if(CGAL_CreateSingleSourceCGALProgram_included)
  return()
endif(CGAL_CreateSingleSourceCGALProgram_included)
set(CGAL_CreateSingleSourceCGALProgram_included TRUE)

include(${CMAKE_CURRENT_LIST_DIR}/CGAL_add_test.cmake)
include(CMakeParseArguments)


add_test(NAME all_cov
  COMMAND ${PYTHON_EXECUTABLE} 
  ${CMAKE_SOURCE_DIR}/Scripts/developer_scripts/cgal_compute_total_coverage.py 
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}/COVERAGE)

function(create_single_source_cgal_program firstfile )
  set(options NO_TESTING)
  set(oneValueArgs)
  set(multiValueArgs CXX_FEATURES)
  cmake_parse_arguments(create_single_source_cgal_program
    "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  set(CXX_FEATURES ${create_single_source_cgal_program_CXX_FEATURES})
  set(NO_TESTING ${create_single_source_cgal_program_NO_TESTING})

  if(NOT IS_ABSOLUTE "${firstfile}")
    set(firstfile "${CMAKE_CURRENT_SOURCE_DIR}/${firstfile}")
  endif()

  get_filename_component(exe_name ${firstfile} NAME_WE)
  
  if(EXISTS "${firstfile}")

    if(CXX_FEATURES AND NOT COMMAND target_compile_features)
      message(STATUS "NOTICE: ${exe_name}.cpp requires a CMake version >= 3.1 to detect C++ features, and will not be compiled.")
      return()
    endif()
    if(CXX_FEATURES)
      set(MISSING_CXX_FEATURES ${CXX_FEATURES})
      if(CMAKE_CXX_COMPILE_FEATURES)
        list(REMOVE_ITEM MISSING_CXX_FEATURES ${CMAKE_CXX_COMPILE_FEATURES})
      endif()
    endif()
    # Now MISSING_CXX_FEATURES is the set CXX_FEATURES minus CMAKE_CXX_COMPILE_FEATURES
    if(MISSING_CXX_FEATURES)
      message(STATUS "NOTICE: ${exe_name}.cpp requires the C++ features \"${MISSING_CXX_FEATURES}\" and will not be compiled.")
      return()
    endif()

    set( all "${firstfile}" )

    # remaining files
    foreach( i ${create_single_source_cgal_program_UNPARSED_ARGUMENTS} )
      set( all ${all} ${CMAKE_CURRENT_SOURCE_DIR}/${i} )
    endforeach()

    add_executable(${exe_name} ${all})
    if(CXX_FEATURES)
      target_compile_features(${exe_name} PRIVATE ${CXX_FEATURES})
    endif()

    get_directory_property(folder_NO_TESTING CGAL_NO_TESTING)

    if(folder_NO_TESTING OR NOT BUILD_TESTING)
      set(NO_TESTING TRUE)
    endif()

    if(POLICY CMP0064)
      # CMake 3.4 or later
      if(NOT NO_TESTING)
        cgal_add_test(${exe_name})
      else()
        cgal_add_test(${exe_name} NO_EXECUTION)
      endif()
    endif()

    add_to_cached_list( CGAL_EXECUTABLE_TARGETS ${exe_name} )

    target_link_libraries(${exe_name} PRIVATE CGAL::CGAL)
    if(CGAL_3RD_PARTY_LIBRARIES)
      target_link_libraries(${exe_name} PRIVATE ${CGAL_3RD_PARTY_LIBRARIES})
    endif()
    if(TESTING_WITH_COVERAGE)
      if(CMAKE_COMPILER_IS_GNUCXX)
        #include(CGAL_CreateTargetForCoverage)
        set(COVERAGE_COMPILER_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
          CACHE INTERNAL "")
        set(CMAKE_CXX_FLAGS_COVERAGE
          ${COVERAGE_COMPILER_FLAGS}
          CACHE STRING "Flags used by the C++ compiler during coverage builds."
          FORCE )
        set(CMAKE_C_FLAGS_COVERAGE
          ${COVERAGE_COMPILER_FLAGS}
          CACHE STRING "Flags used by the C compiler during coverage builds."
          FORCE )
        set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
          ""
          CACHE STRING "Flags used for linking binaries during coverage builds."
          FORCE )
        mark_as_advanced(
          CMAKE_CXX_FLAGS_COVERAGE
          CMAKE_C_FLAGS_COVERAGE
          CMAKE_EXE_LINKER_FLAGS_COVERAGE
          CMAKE_SHARED_LINKER_FLAGS_COVERAGE )
        
        if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
          link_libraries(gcov)
        else()
          set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        endif()        
        
        string(REPLACE "/" ";" PATH_LIST ${firstfile})        
        LIST(GET PATH_LIST -2 package)
        
        if(NOT DEFINED ${package}_DONE)
          add_test(NAME ${package}_pkg_cov
            COMMAND ${PYTHON_EXECUTABLE} 
            ${CMAKE_SOURCE_DIR}/Scripts/developer_scripts/cgal_process_gcov.py 
            ${CMAKE_CURRENT_BINARY_DIR}
            ${CMAKE_SOURCE_DIR}/${package}/package_info/${package}/coverage)
          set_source_files_properties(${CMAKE_BINARY_DIR}/all_cov PROPERTIES GENERATED TRUE )
          set_tests_properties( all_cov PROPERTIES DEPENDS ${package}_pkg_cov)
          set(${package}_DONE TRUE CACHE INTERNAL "")
        endif()
        
        add_test(NAME ${exe_name}_cov
          COMMAND ${CMAKE_COMMAND} 
          -Dexe_name:STRING=${exe_name}
          -DPATH:STRING=${CMAKE_CURRENT_SOURCE_DIR}
          -DGCOV_PATH:STING=${GCOV_PATH}
          -P "${CGAL_MODULES_DIR}/CGAL_generate_reports.cmake")
        set_tests_properties( ${exe_name}_cov PROPERTIES DEPENDS execution___of__${exe_name})
        set_tests_properties( ${package}_pkg_cov PROPERTIES DEPENDS ${exe_name}_cov)
        
       # setup_target_for_coverage(${exe_name} ${package})
      endif()
    endif()

  else()
    message(AUTHOR_WARNING "The executable ${exe_name} will not be created because the source file ${firstfile} does not exist.")
  endif()

endfunction()
