
add_library(${PROJECT_NAME}
            # ${CMAKE_CURRENT_SOURCE_DIR}/config.h
            ${CMAKE_CURRENT_SOURCE_DIR}/System.cc ${CMAKE_CURRENT_SOURCE_DIR}/System.h
            ${CMAKE_CURRENT_SOURCE_DIR}/Tracking.cc ${CMAKE_CURRENT_SOURCE_DIR}/Tracking.h
            ${CMAKE_CURRENT_SOURCE_DIR}/MeshMap.cc ${CMAKE_CURRENT_SOURCE_DIR}/MeshMap.h
        )

# Set output directory of the library
set_target_properties(${PROJECT_NAME} PROPERTIES
                      OUTPUT_NAME STBR
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OPENMP)

# target_compile_definitions(${PROJECT_NAME} PRIVATE pangolin_Viewer)

if(BLAS_FOUND AND LAPACK_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        pangolin_viewer
        ${OpenCV_LIBS}
        Threads::Threads
        opencv_imgcodecs
        opencv_videoio
        opencv_core
        opencv_features2d
        opencv_calib3d
        ${CHOLMOD_LIBRARIES}
        # ${BLAS_LIBRARIES}
        # ${LAPACK_LIBRARIES}
        Eigen3::Eigen
        Open3D::Open3D
        yaml-cpp
        OpenMP::OpenMP_CXX
    )
else()
    message(WARNING "BLAS or LAPACK not found, skipping BLAS/LAPACK linkage")
    target_link_libraries(${PROJECT_NAME} PRIVATE
        pangolin_viewer
        ${OpenCV_LIBS}
        Threads::Threads
        opencv_imgcodecs
        opencv_videoio
        opencv_core
        opencv_features2d
        opencv_calib3d
        ${CHOLMOD_LIBRARIES}
        Eigen3::Eigen
        Open3D::Open3D
        yaml-cpp
        OpenMP::OpenMP_CXX
    )
endif()

# Include directories
target_include_directories(${PROJECT_NAME}
                        PRIVATE
                        # ${Pangolin_INCLUDE_DIR}
                           "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>"
                           "$<INSTALL_INTERFACE:include/>")


set(STBR_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/STBR)

# Install library
install(TARGETS ${PROJECT_NAME}
        EXPORT ${STBR_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STBR_INCLUDE_INSTALL_DIR})

add_subdirectory(optimize)
add_subdirectory(data)
add_subdirectory(GT_compare)
# add_subdirectory(viewer)

