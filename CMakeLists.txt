cmake_minimum_required(VERSION 2.8)
project(camera)

include_directories("/usr/include/eigen3")

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)


set(THIRED_PARTY_LIBS
	${OpenCV_LIBS}
	${Pangolin_LIBRARIES}
	)
#add_executable(stereo_calib_and_rect src/stereo_calib_and_rect.cpp)
#target_link_libraries(stereo_calib_and_rect ${THIRED_PARTY_LIBS})

add_subdirectory(src)
add_subdirectory(app)
