cmake_minimum_required(VERSION 3.2)
project(MeinProjekt)
# cmake_policy(SET CMP0073 NEW)
# Setze die C++ Standard Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -msse4 ") # think on compiler!
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})  
if(POLICY CMP0091)
    # https://stackoverflow.com/a/56490614
    cmake_policy(SET CMP0091 NEW)
endif()

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

find_package(Pangolin REQUIRED)
message(STATUS "Pangolin found")
include_directories(${Pangolin_INCLUDE_DIR})


# Finde die Open3D-Bibliothek
find_package(Open3D REQUIRED)

# Füge den include-Ordner von Open3D hinzu
include_directories(${Open3D_INCLUDE_DIRS})

if(NOT Open3D_FOUND)
    message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
endif()
# Later it has to be deleted ########################
# set(PLOTUTILS_INCLUDE_DIRS /usr/include/plotutils)
# set(PLOTUTILS_LIBRARIES /usr/lib/x86_64-linux-gnu/libplot.so.2.2.4)
# find_package(plotutils REQUIRED)

# Füge die include-Verzeichnisse von plotutils hinzu
# include_directories(${PLOTUTILS_INCLUDE_DIRS})
###########################################

# Füge die ausführbare Datei hinzu
# add_executable(hamlyn app/hamlyn.cc app/objReader.cc app/objReader.h app/helper.cc app/helper.h app/MapViewer.h app/MapViewer.cc)

# # target_include_directories(${PROJECT_NAME}
# #                               PUBLIC
# #                               ${Pangolin_INCLUDE_DIR})

# target_link_libraries(hamlyn PRIVATE 
#                                 ${OpenCV_LIBS}
#                                 opencv_imgcodecs
#                                 opencv_videoio
#                                 opencv_core
#                                 opencv_features2d
#                                 opencv_calib3d
#                                 pangolin
#                                 ${CHOLMOD_LIBRARIES} 
#                                 ${BLAS_LIBRARIES}
#                                 ${LAPACK_LIBRARIES}
#                                 Eigen3::Eigen
#                                 Open3D::Open3D
#                                 # ${PLOTUTILS_LIBRARIES}
#                                 )
# target_link_libraries(${EXECUTABLE_TARGET}
#                 PRIVATE
#                 pangolin_viewer
#                 ${PROJECT_NAME}
#                 #PANGOLIN_VIEWER
#                 Eigen3::Eigen
#                 ${OpenCV_LIBS})
# Verknüpfe Open3D-Bibliothek mit der ausführbaren Datei
# target_link_libraries(hamlyn ${Open3D_LIBRARIES})


#testing area

# add_executable(test app/test.cc app/objReader.cc app/objReader.h app/helper.cc app/helper.h app/MapViewer.h app/MapViewer.cc)

# target_link_libraries(test PRIVATE 
#                                 ${OpenCV_LIBS}
#                                 opencv_imgcodecs
#                                 opencv_videoio
#                                 opencv_core
#                                 opencv_features2d
#                                 opencv_calib3d
#                                 pangolin
#                                 ${CHOLMOD_LIBRARIES} 
#                                 ${BLAS_LIBRARIES}
#                                 ${LAPACK_LIBRARIES}
#                                 Eigen3::Eigen
#                                 Open3D::Open3D
#                                 # ${PLOTUTILS_LIBRARIES}
#                                 )

add_executable(hamlyn_f7 app/hamlyn_f7.cc 
app/Tracking.cc app/Tracking.h
app/MeshMap.cc app/MeshMap.h
app/Mesh_Visualizer.cc app/Mesh_Visualizer.h
app/Extractor.cc app/Extractor.h
app/Optimizer.cc app/Optimizer.h
app/OptimizerDistanceOnly.cc app/OptimizerDistanceOnly.h
)

target_link_libraries(hamlyn_f7 PRIVATE 
                                ${OpenCV_LIBS}
                                opencv_imgcodecs
                                opencv_videoio
                                opencv_core
                                opencv_features2d
                                opencv_calib3d
                                pangolin
                                ${CHOLMOD_LIBRARIES} 
                                ${BLAS_LIBRARIES}
                                ${LAPACK_LIBRARIES}
                                Eigen3::Eigen
                                Open3D::Open3D
                                # ${PLOTUTILS_LIBRARIES}
                                )