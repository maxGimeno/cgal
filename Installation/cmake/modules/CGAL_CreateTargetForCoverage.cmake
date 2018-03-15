include (cat)
find_program( GCOV_PATH gcov )
find_package(PythonInterp 2.6.7)
if(NOT GCOV_PATH)
  message(FATAL_ERROR "gcov not found! Aborting...")
endif(NOT GCOV_PATH)

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
  if("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 3)
    message(FATAL_ERROR "Clang version must be 3.0.0 or greater for coverage support! Aborting...")
  endif()
elseif(NOT CMAKE_COMPILER_IS_GNUCXX)
  message(FATAL_ERROR "Compiler is not GNU gcc! Coverage not supported. Aborting.")
endif()

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

  
function(SETUP_TARGET_FOR_COVERAGE Coverage pkg)
  
  set(options NONE)
  cmake_parse_arguments(Coverage "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  
  #add custom commands 
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${Coverage}_report.txt
    COMMAND ${Coverage}
    COMMAND ${GCOV_PATH} ${Coverage}.cpp -o ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${Coverage}.dir/${Coverage}.cpp.gcno > ${CMAKE_CURRENT_BINARY_DIR}/${Coverage}_report.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${Coverage} 
    )
  
  add_custom_command( OUTPUT ${CMAKE_SOURCE_DIR}/${pkg}/package_info/${pkg}/coverage
            COMMAND ${PYTHON_EXECUTABLE} 
            ${CMAKE_SOURCE_DIR}/Scripts/developer_scripts/cgal_process_gcov.py 
            ${CMAKE_BINARY_DIR}/test/${pkg} 
            ${CMAKE_SOURCE_DIR}/${pkg}/package_info/${pkg}/coverage
    )
  
  add_custom_command( OUTPUT ${CMAKE_BINARY_DIR}/COVERAGE
            COMMAND ${PYTHON_EXECUTABLE} 
            ${CMAKE_SOURCE_DIR}/Scripts/developer_scripts/cgal_compute_total_coverage.py 
            ${CMAKE_SOURCE_DIR}
            ${CMAKE_BINARY_DIR}/COVERAGE
    )
  
  # Setup target
  add_custom_target(${Coverage}_coverage
    # Run tests
    COMMAND ${CMAKE_COMMAND} -E remove ${Coverage}_coverage.base ${Coverage}_coverage.info ${Coverage}_coverage.total ${PROJECT_BINARY_DIR}/${Coverage}_coverage.info.cleaned
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${Coverage}_report.txt 
    )
  if ( NOT TARGET all_cov)
    if(PYTHONINTERP_FOUND)
      add_custom_target(all_cov
        COMMAND cat "${CMAKE_BINARY_DIR}/COVERAGE"
        DEPENDS "${CMAKE_BINARY_DIR}/COVERAGE"
        )
    endif()
  endif()
  if ( NOT TARGET ${pkg}_cov)
    if(PYTHONINTERP_FOUND)
      add_custom_target(${pkg}_cov
        COMMAND cat "${CMAKE_SOURCE_DIR}/${pkg}/package_info/${pkg}/coverage"
        DEPENDS "${CMAKE_SOURCE_DIR}/${pkg}/package_info/${pkg}/coverage"
        )
      add_dependencies( all_cov ${pkg}_cov )
    endif()
  endif()
  add_dependencies( ${pkg}_cov ${Coverage}_coverage)
  
endfunction() # SETUP_TARGET_FOR_COVERAGE
