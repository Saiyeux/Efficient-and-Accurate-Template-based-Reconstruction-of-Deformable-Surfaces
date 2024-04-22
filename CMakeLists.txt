cmake_minimum_required(VERSION 3.22)
project(Sparse_Template_Based_Reconstruction)

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

find_package(OpenCV 4.5.0 REQUIRED)
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
find_package(Open3D 0.18.0 REQUIRED)
message(STATUS "Open3D found")

# Füge den include-Ordner von Open3D hinzu
include_directories(${Open3D_INCLUDE_DIRS})

if(NOT Open3D_FOUND)
    message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
endif()

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# Install to the specified directory
# install(FILES ${STBR_VERSION_CONFIG} ${STBR_PROJECT_CONFIG}
#         DESTINATION ${STBR_CONFIG_INSTALL_DIR})

# install(EXPORT ${STBR_TARGETS_EXPORT_NAME}
#         NAMESPACE ${STBR_NAMESPACE}
#         DESTINATION ${STBR_CONFIG_INSTALL_DIR})

# Set standard installation directories
set(RUNTIME_DESTINATION bin)
set(LIBRARY_DESTINATION lib)
set(ARCHIVE_DESTINATION lib)
set(INCLUDES_DESTINATION include)

add_subdirectory(src)
add_subdirectory(app)

add_subdirectory(test)

