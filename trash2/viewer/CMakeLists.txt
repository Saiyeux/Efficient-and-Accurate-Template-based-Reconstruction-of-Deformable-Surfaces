


add_library(pangolin_Viewer
            # ${CMAKE_CURRENT_SOURCE_DIR}/config.h
                MeshViewer.cc MeshViewer.h
                # MeshDrawer.cc MeshDrawer.h
            )

# Set output directory of the library
set_target_properties(pangolin_Viewer PROPERTIES
                      OUTPUT_NAME pangolin_Viewer
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# It has to be PUBLIC not PRIVATE!!!!
target_link_libraries(pangolin_Viewer 
                        PUBLIC
                        # ${PROJECT_NAME}
                        ${OpenCV_LIBS}
                        opencv_imgcodecs
                        opencv_videoio
                        opencv_core
                        opencv_features2d
                        opencv_calib3d
                        
                        #pangolin
                        # ${CHOLMOD_LIBRARIES} 
                        # ${BLAS_LIBRARIES}
                        # ${LAPACK_LIBRARIES}
                        Eigen3::Eigen
                        # Open3D::Open3D
                        yaml-cpp
                        PUBLIC
                        pango_display
                        pango_opengl
                        ${Pangolin_LIBRARIES}
                        # ${PLOTUTILS_LIBRARIES}
                        )

# Include directories
target_include_directories(pangolin_Viewer
                                PRIVATE
                        #    ${Pangolin_INCLUDE_DIR}
                           "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/viewer/>"
                           "$<INSTALL_INTERFACE:include/viewer>")

set(PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/pangolin_Viewer)

# Install library
install(TARGETS pangolin_Viewer
        EXPORT ${PANGOLIN_VIEWER_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})
        # INCLUDES DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})



# Fügen Sie Ihre Quelldateien hinzu
# add_executable(mein_programm view.cc)
# target_compile_definitions(mein_programm PRIVATE pangolin_Viewer)
# Verknüpfen Sie Ihre Anwendung mit der Pangolin-Bibliothek
# target_link_libraries(mein_programm PUBLIC pangolin_Viewer)


# # Add sources
# target_sources(${PROJECT_NAME}
#                PRIVATE
#                ${CMAKE_CURRENT_SOURCE_DIR}/MeshDrawer.cc ${CMAKE_CURRENT_SOURCE_DIR}/MeshDrawer.h
#                ${CMAKE_CURRENT_SOURCE_DIR}/MeshViewer.cc ${CMAKE_CURRENT_SOURCE_DIR}/MeshViewer.h
#                )

# # Install headers
# file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
# install(FILES ${HEADERS}
#         DESTINATION ${STBR_INCLUDE_INSTALL_DIR}/viewer)


# Fügen Sie Ihre Quelldateien hinzu
# add_executable(mein_programm view.cc)
# target_compile_definitions(mein_programm PRIVATE pangolin_Viewer)
# # Verknüpfen Sie Ihre Anwendung mit der Pangolin-Bibliothek
# target_link_libraries(mein_programm PUBLIC pangolin_Viewer)