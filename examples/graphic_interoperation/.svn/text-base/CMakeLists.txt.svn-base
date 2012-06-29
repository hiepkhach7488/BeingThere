#Kinect Capture
FILE(GLOB header *.h*)
FILE(GLOB source *.cpp *.cu)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${Beingthere_INCLUDE_DIR}
	${Kinfu_INCLUDE_DIR}
)

set(the_target graphics_interoperation)
CUDA_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${header} ${source})

target_link_libraries(${the_target} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OPENNI_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES} beingthere_kinfu)  

