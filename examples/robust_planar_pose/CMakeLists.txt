#Kinect Capture
FILE(GLOB header *.h*)
FILE(GLOB source *.cpp)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
)

set(the_target robust_planar_pose)
ADD_EXECUTABLE(${the_target} ${header} ${source})

target_link_libraries(${the_target} ${OpenCV_LIBS})  

