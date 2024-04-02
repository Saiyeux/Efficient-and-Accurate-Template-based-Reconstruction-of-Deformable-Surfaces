# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/GroundTruth_compare.cc ${CMAKE_CURRENT_SOURCE_DIR}/GroundTruth_compare.h
               ${CMAKE_CURRENT_SOURCE_DIR}/HamlynGT.cc ${CMAKE_CURRENT_SOURCE_DIR}/HamlynGT.h
               )

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STBR_INCLUDE_INSTALL_DIR}/GT_compare)