add_library(sm_lib SHARED
				chessboard_images.cpp
				stereo_calib_and_rect.cpp
				stereo_vision.cpp
			)

target_link_libraries(sm_lib ${THIRED_PARTY_LIBS})
