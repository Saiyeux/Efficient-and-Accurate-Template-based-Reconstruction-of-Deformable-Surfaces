add_library(${PROJECT_NAME}
            # ${CMAKE_CURRENT_SOURCE_DIR}/config.h
            Tracking.cc Tracking.h
            MeshMap.cc MeshMap.h
            Mesh_Visualizer.cc Mesh_Visualizer.h
            Extractor.cc Extractor.h
            Optimizer.cc Optimizer.h
            OptimizerWithoutMiddlePoint.cc OptimizerWithoutMiddlePoint.h
            OptimizerDistanceOnly.cc OptimizerDistanceOnly.h)

# Set output directory of the library
set_target_properties(${PROJECT_NAME} PROPERTIES
                      OUTPUT_NAME STBR
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)


target_link_libraries(${PROJECT_NAME} PRIVATE 
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
                                yaml-cpp
                                # ${PLOTUTILS_LIBRARIES}
                                )

# Include directories
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>"
                           "$<INSTALL_INTERFACE:include/>")


set(STBR_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/STBR)

# Install OpenVSLAM library
install(TARGETS ${PROJECT_NAME}
        EXPORT ${STBR_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# Install OpenVSLAM headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STBR_INCLUDE_INSTALL_DIR})