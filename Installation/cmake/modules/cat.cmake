function(CAT file)
  message("READ THE FUCKING FILE MOTHERFUCKER !")
  FILE(READ ${file} lines)
  message("READ THE FUCKING FILE MOTHERFUCKER !")
  foreach(line ${lines})
    message ("${line}")
  endforeach()
endfunction()
