  execute_process(
    COMMAND ${GCOV_PATH} "${PATH}/${exe_name}.cpp" -o ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${exe_name}.dir/${exe_name}.cpp.gcno
    WORKING_DIRECTOR ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/${exe_name}_report.txt
    ERROR_VARIABLE error
    )
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${exe_name}_report.err "${error} \n")
  
