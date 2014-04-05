cmake_minimum_required(VERSION 2.6)

#find required libraries
FIND_PACKAGE(position_3d REQUIRED)
FIND_PACKAGE(observations REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# add the necessary include directories
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(${position_3d_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${observations_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/eigen3)

# geometry headers
SET(geometry_headers
      point.h
      line.h
)

# geometry sources
SET(geometry_sources
      point.cpp
      line.cpp
)

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

# random utility header files
SET(random_headers
      random/simpleRnd.h)

# random utility source files
SET(random_sources
      random/simpleRnd.cpp)	
	
# appearance header files
SET(appearance_headers
      appearance/histogram.h
      appearance/hogmodel.h
      appearance/colorBoxes.h
      appearance/hsHistogram.h
      appearance/rgbHistogram.h
      appearance/normalizedpatchmodel.h
      appearance/simplepatchmodel.h
      appearance/covariancemodel.h
      appearance/appearance.h
      appearance/combined1.h
      appearance/oldam.h
)

# appearance source files
SET(appearance_sources
        appearance/histogram.cpp
        appearance/hogmodel.cpp
        appearance/colorBoxes.cpp
        appearance/hsHistogram.cpp
        appearance/rgbHistogram.cpp
        appearance/normalizedpatchmodel.cpp
        appearance/simplepatchmodel.cpp
        appearance/covariancemodel.cpp
        appearance/appearance.cpp
        appearance/combined1.cpp
        appearance/oldam.cpp
)

# create the shared library
ADD_LIBRARY(people_tracking_rai SHARED ${geometry_sources} ${tracker_sources} ${random_sources} ${appearance_sources})

# link necessary libraries
TARGET_LINK_LIBRARIES(people_tracking_rai ${position_3d_LIBRARY})
TARGET_LINK_LIBRARIES(people_tracking_rai ${observations_LIBRARY})
TARGET_LINK_LIBRARIES(people_tracking_rai ${OpenCV_LIBS})

# install 
INSTALL(TARGETS people_tracking_rai
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)
        
INSTALL(FILES ${geometry_headers} DESTINATION include/iridrivers)
INSTALL(FILES ${tracker_headers} DESTINATION include/iridrivers)
INSTALL(FILES ${random_headers} DESTINATION include/iridrivers/random)
INSTALL(FILES ${appearance_headers} DESTINATION include/iridrivers/appearance)
INSTALL(FILES ../Findpeople_tracking_rai.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

ADD_SUBDIRECTORY(examples)
