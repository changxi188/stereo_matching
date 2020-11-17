add_executable(collect_image collect_image.cpp)
target_link_libraries(collect_image ${THIRED_PARTY_LIBS})

add_executable(start_stereo_match start_stereo_match.cpp)
target_link_Libraries(start_stereo_match sm_lib ${THIRED_PARTY_LIBS})

