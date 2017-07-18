## Version: $Id$
##
######################################################################
##
### Commentary:
##
######################################################################
##
### Change Log:
##
######################################################################
##
### Code:

set(Blaze_INCLUDE_SEARCH_PATHS
  /usr/include
  /usr/include/blaze/
  /usr/local/include/
  /opt/blaze/
  $ENV{Blaze_DIR}/
  )

find_path(Blaze_INCLUDE_DIR
  NAMES blaze/Blaze.h
  PATHS ${Blaze_INCLUDE_SEARCH_PATHS})

include (FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(Blaze
  FOUND_VAR Blaze_FOUND
  REQUIRED_VARS Blaze_INCLUDE_DIR
  )

if ( Blaze_FOUND )
  MESSAGE(  "Blaze found: header(${Blaze_INCLUDE_DIR})" )
endif()

mark_as_advanced( Blaze_INCLUDE_DIR )

######################################################################
### FindBlaze.cmake ends here
