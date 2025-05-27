cmake_minimum_required(VERSION 3.22)
project(Sparse_Template_Based_Reconstruction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -msse4 ") # think on compiler!
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})  

set(BLAS_DIR /usr/local/lib)
set(LAPACK_DIR /usr/local/lib)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/local/lib)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /usr/local)

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

if(POLICY CMP0091)
    # https://stackoverflow.com/a/56490614
    cmake_policy(SET CMP0091 NEW)
endif()

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHCXX17)
   message(STATUS "Using flag -std=c++17.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# Add the directories containing the header files to the include path
#include_directories(${PROJECT_SOURCE_DIR}/src1)
# include_directories(${PROJECT_SOURCE_DIR}/src/data)
# include_directories(${PROJECT_SOURCE_DIR}/src/GT_compare)
# include_directories(${PROJECT_SOURCE_DIR}/src/optimize)
include_directories(${PROJECT_SOURCE_DIR}/src/viewer)
include_directories(${PROJECT_SOURCE_DIR}/src/modules)
include_directories(${PROJECT_SOURCE_DIR}/src/modules/data)
include_directories(${PROJECT_SOURCE_DIR}/src/modules/GT_compare)
include_directories(${PROJECT_SOURCE_DIR}/src/modules/optimize)
include_directories(${PROJECT_SOURCE_DIR}/src/util)
include_directories(${PROJECT_SOURCE_DIR}/test)

# include_directories(${PROJECT_SOURCE_DIR}/app)
# include_directories(${PROJECT_SOURCE_DIR}/viewer)

find_package(CUDA REQUIRED)
message("CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARIES})


# find_package(BLAS REQUIRED)
# find_package(LAPACK REQUIRED)
if(BLAS_FOUND AND LAPACK_FOUND)
    message(STATUS "Found BLAS: ${BLAS_LIBRARIES}")
    message(STATUS "Found LAPACK: ${LAPACK_LIBRARIES}")
else()
    message("BLAS or LAPACK not found")
endif()

find_package(Cholmod)
if(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
    message(STATUS "Found CHOLMOD and its dependencies")
    set(CHOLMOD_FOUND TRUE)
else()
    message(WARNING "CHOLMOD or its dependencies (BLAS/LAPACK) not found")
    set(CHOLMOD_FOUND FALSE)
endif()
include_directories(${CHOLMOD_INCLUDE_DIR})

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Use Eigen")

find_package(Pangolin 0.9 REQUIRED)
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

# OpenMP
find_package(OpenMP REQUIRED)

# find_package(Pangolin 0.8 REQUIRED)
# if(NOT Pangolin_FOUND)
#     message(FATAL_ERROR "Pangolin not found!")
# endif()

# Fügen Sie den Include-Verzeichnis von Pangolin hinzu
# include_directories(${Pangolin_INCLUDE_DIRS})

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

# add_subdirectory(src)

add_subdirectory(src)
add_subdirectory(app)
# add_subdirectory(test)


# add_subdirectory(${PROJECT_SOURCE_DIR}/src/viewer)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/optimize)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/data)
# add_subdirectory(${PROJECT_SOURCE_DIR}/src/GT_compare)
# add_subdirectory(${PROJECT_SOURCE_DIR}/app)

