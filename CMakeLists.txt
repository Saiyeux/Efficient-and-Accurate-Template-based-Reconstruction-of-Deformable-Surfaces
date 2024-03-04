cmake_minimum_required(VERSION 3.1)
project(MeinProjekt)

# Setze die C++ Standard Version
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
add_executable(hamlyn app/hamlyn.cc)
target_link_libraries(hamlyn ${OpenCV_LIBS}
                                opencv_imgcodecs
                                opencv_videoio
                                opencv_core
                                opencv_features2d
                                opencv_calib3d
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
