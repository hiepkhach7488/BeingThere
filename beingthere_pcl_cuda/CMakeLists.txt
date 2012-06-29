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

FILE(GLOB cuda src/*.cu)
# set(cuda
	# src/disparity_to_cloud.cu
	# src/host_device.cu
	# src/extract_indices.cu
	# src/debayering.cu
	# src/normal_3d.cu
	# )
	
#set(srcs src/cloud_to_pcl.cpp)

FILE(GLOB srcs src/*.cpp)

set(incs
	include/pcl/cuda/io/cloud_from_pcl.h
	include/pcl/cuda/io/cloud_to_pcl.h
	include/pcl/cuda/io/debayering.h
	include/pcl/cuda/io/disparity_to_cloud.h
	include/pcl/cuda/io/extract_indices.h
	include/pcl/cuda/io/host_device.h
	include/pcl/cuda/io/predicate.h
	include/pcl/cuda/features/normal_3d_kernels.h
	include/pcl/cuda/features/normal_3d.h
	)

source_group("Source Files\\cuda" FILES ${srcs} ${cuda} )    
source_group("Source Files" FILES ${incs} )

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

SET(LIB_NAME beingthere_pcl_cuda)
ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${cuda} ${cuda_objs})
target_link_libraries(${LIB_NAME} ${PCL_GPU_UTILS_LIBRARIES} ${PCL_GPU_CONTAINERS_LIBRARIES} 
${PCL_GPU_OCTREE_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES})

if(OpenCV_FOUND)
	target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})
endif()

