cmake_minimum_required(VERSION 2.6)

#find required libraries
FIND_PACKAGE(OpenCV REQUIRED)

# add the necessary include directories
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES($ENV{EIGEN3_INCLUDE_DIR})

# tracker header files
SET(tracker_headers 
      personParticle.h
      personParticleFilter.h
      personTarget.h
      peopleTracker.h)
	
# tracker source files
SET(tracker_sources 
	personParticle.cpp
      personParticleFilter.cpp
      personTarget.cpp
	peopleTracker.cpp)	

# geometry headers
SET(geometry_headers
      geometry/point.h
      geometry/line.h
      geometry/point3d.h      
      geometry/point3dCov.h      
)

# geometry sources
SET(geometry_sources
      geometry/point.cpp
      geometry/line.cpp
      geometry/point3d.cpp      
      geometry/point3dCov.cpp
)

# observations headers
SET(observations_headers
      observations/timeStamp.h
      observations/basicObservation.h
      observations/odometryObservation.h
      observations/point3dObservation.h
      observations/bodyObservation.h
      observations/faceObservation.h
)

# observations sources
SET(observations_sources
      observations/timeStamp.cpp
      observations/basicObservation.cpp
      observations/odometryObservation.cpp
      observations/point3dObservation.cpp
      observations/bodyObservation.cpp
      observations/faceObservation.cpp
)
		
# random utility header files
SET(random_headers
      random/simpleRnd.h)

# random utility source files
SET(random_sources
      random/simpleRnd.cpp)	
	
# appearance header files
# SET(appearance_headers
#       appearance/histogram.h
#       appearance/hogmodel.h
#       appearance/colorBoxes.h
#       appearance/hsHistogram.h
#       appearance/rgbHistogram.h
#       appearance/normalizedpatchmodel.h
#       appearance/simplepatchmodel.h
#       appearance/covariancemodel.h
#       appearance/appearance.h
#       appearance/combined1.h
#       appearance/oldam.h
# )

# appearance source files
# SET(appearance_sources
#       appearance/histogram.cpp
#       appearance/hogmodel.cpp
#       appearance/colorBoxes.cpp
#       appearance/hsHistogram.cpp
#       appearance/rgbHistogram.cpp
#       appearance/normalizedpatchmodel.cpp
#       appearance/simplepatchmodel.cpp
#       appearance/covariancemodel.cpp
#       appearance/appearance.cpp
#       appearance/combined1.cpp
#       appearance/oldam.cpp
# )

# create the shared library
# ADD_LIBRARY(pipol_tracker SHARED ${geometry_sources} ${observations_sources} ${tracker_sources} ${random_sources} ${appearance_sources})
ADD_LIBRARY(pipol_tracker SHARED ${geometry_sources} ${observations_sources} ${tracker_sources} ${random_sources})

# link necessary libraries
TARGET_LINK_LIBRARIES(pipol_tracker ${OpenCV_LIBS})

# install 
INSTALL(TARGETS pipol_tracker
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/btr-libs
        ARCHIVE DESTINATION lib/btr-libs)
        
INSTALL(FILES ${tracker_headers} DESTINATION include/btr-headers)
INSTALL(FILES ${geometry_headers} DESTINATION include/btr-headers/geometry)
INSTALL(FILES ${observations_headers} DESTINATION include/btr-headers/observations)
INSTALL(FILES ${random_headers} DESTINATION include/btr-headers/random)
# INSTALL(FILES ${appearance_headers} DESTINATION include/btr-headers/appearance)
INSTALL(FILES ../Findpipol_tracker.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

ADD_SUBDIRECTORY(examples)
