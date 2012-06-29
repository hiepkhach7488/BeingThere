#Kinect Capture
FILE(GLOB header *.h*)
FILE(GLOB source *.cpp)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${Beingthere_INCLUDE_DIR}
	${Kinfu_INCLUDE_DIR}
)

source_group("Source Files" FILES ${source} )
source_group("Header Files" FILES ${header} )

set(the_target multicam_fusion)
CUDA_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${header} ${source})

target_link_libraries(${the_target} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OPENNI_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES} beingthere_kinfu)  

