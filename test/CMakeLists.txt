# Suchen Sie Pangolin und stellen Sie sicher, dass es gefunden wurde
find_package(Pangolin 0.8 REQUIRED)
if(NOT Pangolin_FOUND)
    message(FATAL_ERROR "Pangolin not found!")
endif()

# Fügen Sie den Include-Verzeichnis von Pangolin hinzu
include_directories(${Pangolin_INCLUDE_DIRS})

# Fügen Sie Ihre Quelldateien hinzu
add_executable(mein_programm view.cc)

# Verknüpfen Sie Ihre Anwendung mit der Pangolin-Bibliothek
target_link_libraries(mein_programm ${Pangolin_LIBRARIES})


add_library(MeshViewer
            # ${CMAKE_CURRENT_SOURCE_DIR}/config.h
            MeshViewer.cc MeshViewer.h
            )

# Set output directory of the library
set_target_properties(MeshViewer PROPERTIES
                      OUTPUT_NAME MeshViewer
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)


target_link_libraries(MeshViewer PRIVATE 
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
                                ${Pangolin_LIBRARIES}
                                # ${PLOTUTILS_LIBRARIES}
                                )

# Include directories
target_include_directories(MeshViewer
                           PUBLIC
                           "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Viewer/>"
                           "$<INSTALL_INTERFACE:include/>")


set(STBR_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/MeshViewer)

# Install OpenVSLAM library
install(TARGETS MeshViewer
        EXPORT ${STBR_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STBR_INCLUDE_INSTALL_DIR})