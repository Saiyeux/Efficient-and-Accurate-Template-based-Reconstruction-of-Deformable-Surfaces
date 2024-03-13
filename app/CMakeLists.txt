set(EXECUTABLE_TARGETS "")

add_executable(hamlyn_f7 hamlyn_f7.cc)
list(APPEND EXECUTABLE_TARGETS hamlyn_f7)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")




    # Link OpenVSLAM
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE
                          ${PROJECT_NAME}
                          ${OpenCV_LIBS}
                          Open3D::Open3D
                          yaml-cpp
                          opencv_imgcodecs
                          opencv_videoio)

endforeach()