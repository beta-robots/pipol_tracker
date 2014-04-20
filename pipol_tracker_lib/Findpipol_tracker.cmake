FIND_PATH(
      pipol_tracker_INCLUDE_DIRS
      NAMES 
            personParticle.h personParticleFilter.h personTarget.h peopleTracker.h
            geometry/point.h geometry/line.h geometry/point3d.h geometry/point3dCov.h      
            observations/timeStamp.h observations/basicObservation.h observations/odometryObservation.h observations/point3dObservation.h observations/bodyObservation.h observations/faceObservation.h
            random/simpleRnd.h
            
      PATHS /usr/local/include/btr-headers
)

FIND_LIBRARY(
      pipol_tracker_LIBRARIES
      NAMES pipol_tracker
      PATHS /usr/local/lib/btr-libs
)


IF (pipol_tracker_INCLUDE_DIRS)
      MESSAGE(STATUS "------------ Found pipol_tracker_INCLUDE_DIRS: ${pipol_tracker_INCLUDE_DIRS}")
ELSE (pipol_tracker_INCLUDE_DIRS)
      MESSAGE(STATUS "------------ NOT Found pipol_tracker_INCLUDE_DIRS: ${pipol_tracker_INCLUDE_DIRS}")
ENDIF (pipol_tracker_INCLUDE_DIRS)


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

