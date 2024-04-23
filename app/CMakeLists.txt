set(EXECUTABLE_TARGETS "")

add_executable(colon_1 colon_1/main.cc)
list(APPEND EXECUTABLE_TARGETS colon_1)

add_executable(hamlyn_f7 Phantom7/hamlyn_f7.cc)
list(APPEND EXECUTABLE_TARGETS hamlyn_f7)

add_executable(hamlyn_f5 Phantom5/hamlyn_f5.cc)
list(APPEND EXECUTABLE_TARGETS hamlyn_f5)

add_executable(S1_real p_sft_S1_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S1_real)

add_executable(S2_real p_sft_S2_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S2_real)

add_executable(S3_real p_sft_S3_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S3_real)

add_executable(S4_real p_sft_S4_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S4_real)

add_executable(S5_real p_sft_S5_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S5_real)

add_executable(S6_real p_sft_S6_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S6_real)

add_executable(S7_real p_sft_S7_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S7_real)

add_executable(S8_real p_sft_S8_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S8_real)

add_executable(S9_real p_sft_S9_real/real.cc)
list(APPEND EXECUTABLE_TARGETS S9_real)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")




    # Link Executable
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE
                          ${PROJECT_NAME}
                          ${OpenCV_LIBS}
                          opencv_imgcodecs
                                opencv_videoio
                                opencv_core
                                opencv_features2d
                                opencv_calib3d
                          Open3D::Open3D
                          yaml-cpp)
                        #   opencv_imgcodecs
                        #   opencv_videoio)

endforeach()