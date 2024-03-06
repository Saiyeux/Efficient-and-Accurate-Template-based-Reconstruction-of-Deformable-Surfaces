cmake_minimum_required(VERSION 3.1)
project(MeinProjekt)

# Setze die C++ Standard Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -msse4 ") # think on compiler!
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})  


find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
find_package(GOTOBLAS)
IF(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  MESSAGE(STATUS "Found CHOLMOD and its dependencies")
  SET(CHOLMOD_FOUND TRUE)
ELSE()
  SET(CHOLMOD_FOUND FALSE)
ENDIF()
include_directories(${CHOLMOD_INCLUDE_DIR})
# find_package(OpenCV 3.3.1 QUIET
#              COMPONENTS
#              core imgcodecs videoio features2d calib3d highgui)
# if(NOT OpenCV_FOUND)
#     find_package(OpenCV 4.0 QUIET
#                  COMPONENTS
#                  core imgcodecs videoio features2d calib3d highgui)
#     if(NOT OpenCV_FOUND)
#         message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
#     endif()
# endif()
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Use Eigen")

# Finde die Open3D-Bibliothek
# find_package(Open3D REQUIRED)

# Füge den include-Ordner von Open3D hinzu
# include_directories(${Open3D_INCLUDE_DIRS})

# Füge die ausführbare Datei hinzu
add_executable(hamlyn app/hamlyn.cc app/objReader.cc app/objReader.h app/helper.cc app/helper.h)
target_link_libraries(hamlyn PRIVATE 
                                ${OpenCV_LIBS}
                                opencv_imgcodecs
                                opencv_videoio
                                opencv_core
                                opencv_features2d
                                opencv_calib3d
                                ${CHOLMOD_LIBRARIES} 
                                ${BLAS_LIBRARIES}
                                ${LAPACK_LIBRARIES}
                                Eigen3::Eigen)
# target_link_libraries(${EXECUTABLE_TARGET}
#                 PRIVATE
#                 pangolin_viewer
#                 ${PROJECT_NAME}
#                 #PANGOLIN_VIEWER
#                 Eigen3::Eigen
#                 ${OpenCV_LIBS})
# Verknüpfe Open3D-Bibliothek mit der ausführbaren Datei
# target_link_libraries(hamlyn ${Open3D_LIBRARIES})
