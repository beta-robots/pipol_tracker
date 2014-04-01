FIND_PATH(
      pipol_tracker_INCLUDE_DIRS
      NAMES *.h
      PATHS /usr/local/include/btr-headers
)

FIND_LIBRARY(
      pipol_tracker_LIBRARIES
      NAMES pipol_tracker
      PATHS /usr/local/lib/btr-libs
)

IF (pipol_tracker_INCLUDE_DIRS AND pipol_tracker_LIBRARIES)
   SET(pipol_tracker_FOUND TRUE)
ENDIF (pipol_tracker_INCLUDE_DIRS AND pipol_tracker_LIBRARIES)

IF (pipol_tracker_FOUND)
   IF (NOT pipol_tracker_FIND_QUIETLY)
      MESSAGE(STATUS "Found pipol_tracker library: ${pipol_tracker_LIBRARIES}")
   ENDIF (NOT pipol_tracker_FIND_QUIETLY)
ELSE (pipol_tracker_FOUND)
   IF (pipol_tracker_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find pipol_tracker library")
   ENDIF (pipol_tracker_FIND_REQUIRED)
ENDIF (pipol_tracker_FOUND)

