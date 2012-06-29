#POSE_ESTIMATION
set(the_target pose_estimation)
FILE(GLOB srcs *.cpp)
FILE(GLOB hdrs *.h*)

include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}
		${Beingthere_INCLUDE_DIR}
		${Kinfu_INCLUDE_DIR}
	)

source_group("Source Files" FILES ${srcs} )
source_group("Header Files" FILES ${hdrs} )

CUDA_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})

if(OpenCV_FOUND)
	target_link_libraries(${the_target} ${OpenCV_LIBS})
endif()

target_link_libraries(${the_target} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OPENNI_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES} beingthere_kinfu beingthere_io)  

