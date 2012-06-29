include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(OpenCV_FOUND)
	add_definitions(-DHAVE_OPENCV)	
	include_directories(${OpenCV_INCLUDE_DIRS})
	link_directories(${OpenCV_LIB_DIR})
	link_libraries(${OpenCV_LIBS})
else()
	remove_definitions(-DHAVE_OPENCV)
endif()
	
FILE(GLOB incs include/pcl/gpu/kinfu/*.h*)
FILE(GLOB srcs src/*.cpp src/*.h*)
FILE(GLOB cuda src/cuda/*.cu src/cuda/*.h*)

source_group("Source Files\\cuda" FILES ${cuda} )    
source_group("Source Files" FILES ${srcs} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src ${CUDA_INCLUDE_DIRS})			

if (UNIX OR APPLE)
	set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")			
endif()

if(NOT UNIX OR APPLE)
	add_definitions(-DPCLAPI_EXPORTS)
endif()

if(TRUE)			   
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") 
	CUDA_COMPILE(cuda_objs ${cuda})		
else()		

endif()

SET(LIB_NAME beingthere_pcl_kinfu)
ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${cuda} ${cuda_objs})
target_link_libraries(${LIB_NAME} ${PCL_GPU_UTILS_LIBRARIES} ${PCL_GPU_CONTAINERS_LIBRARIES} 
${PCL_GPU_OCTREE_LIBRARIES})

if(OpenCV_FOUND)
	target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})
endif()

