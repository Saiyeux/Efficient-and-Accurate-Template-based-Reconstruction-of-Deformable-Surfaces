# # Suchen Sie Pangolin und stellen Sie sicher, dass es gefunden wurde
# find_package(Pangolin 0.9 REQUIRED)
# if(NOT Pangolin_FOUND)
#     message(FATAL_ERROR "Pangolin not found!")
# endif()

# # Fügen Sie den Include-Verzeichnis von Pangolin hinzu
# include_directories(${Pangolin_INCLUDE_DIRS})


# add_library(pangolin_Viewer1
#             # ${CMAKE_CURRENT_SOURCE_DIR}/config.h
#             Viewer.cc Viewer.h
#             MeshDrawer.cc MeshDrawer.h
#             )

# # Set output directory of the library
# set_target_properties(pangolin_Viewer1 PROPERTIES
#                       OUTPUT_NAME pangolin_Viewer1
#                       ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
#                       LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
# # Include directories
# target_include_directories(pangolin_Viewer1
#                            PUBLIC
#                            ${Pangolin_INCLUDE_DIR}
#                            "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Viewer1/>"
#                            "$<INSTALL_INTERFACE:include/>")

# # It has to be PUBLIC not PRIVATE!!!!
# target_link_libraries(pangolin_Viewer1 PUBLIC 
#                                 pango_display
#                                 pango_opengl
#                                 ${OpenCV_LIBS}
#                                 opencv_imgcodecs
#                                 opencv_videoio
#                                 opencv_core
#                                 opencv_features2d
#                                 opencv_calib3d
#                                 #pangolin
#                                 # ${CHOLMOD_LIBRARIES} 
#                                 # ${BLAS_LIBRARIES}
#                                 # ${LAPACK_LIBRARIES}
#                                 Eigen3::Eigen
#                                 # Open3D::Open3D
#                                 yaml-cpp
#                                 ${Pangolin_LIBRARIES}
#                                 # ${PLOTUTILS_LIBRARIES}
#                                 )

# set(MESH_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/pangolin_Viewer1)

# # Install library
# install(TARGETS pangolin_Viewer1
#         EXPORT ${MESH_VIEWER_TARGETS_EXPORT_NAME}
#         RUNTIME DESTINATION ${RUNTIME_DESTINATION}
#         LIBRARY DESTINATION ${LIBRARY_DESTINATION}
#         ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# # Install headers
# file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
# install(FILES ${HEADERS}
#         DESTINATION ${MESH_VIEWER_INCLUDE_INSTALL_DIR})

set(EXECUTABLE_TARGETS "")

add_executable(mein_programm view.cc)
list(APPEND EXECUTABLE_TARGETS mein_programm)

add_executable(mein_programm2 p_sft_S1_real/real.cc)
list(APPEND EXECUTABLE_TARGETS mein_programm2)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
                set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

        # target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE pangolin_viewer)
        # Verknüpfen Sie Ihre Anwendung mit der Pangolin-Bibliothek
        target_link_libraries(${EXECUTABLE_TARGET} PUBLIC 
                                # Open3D::Open3D
                                #  PRIVATE 
                                # pangolin_viewer
                                
                                ${PROJECT_NAME}
                                ${OpenCV_LIBS}
                                util_files
                                # Threads::Threads
                                # opencv_imgcodecs
                                # opencv_videoio
                                # opencv_core
                                # opencv_features2d
                                # opencv_calib3d
                                # pangolin
                                # ${CHOLMOD_LIBRARIES} 
                                # ${BLAS_LIBRARIES}
                                # ${LAPACK_LIBRARIES}
                                # Eigen3::Eigen

                                
                                yaml-cpp
                        
                                )
endforeach()
# Fügen Sie Ihre Quelldateien hinzu
# add_executable(mein_programm view.cc)
# target_compile_definitions(mein_programm PRIVATE pangolin_Viewer1)
# # Verknüpfen Sie Ihre Anwendung mit der Pangolin-Bibliothek
# target_link_libraries(mein_programm PUBLIC pangolin_Viewer1)


