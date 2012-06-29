cmake_minimum_required(VERSION 2.8)

project(BEING_THERE)

#FIND CUDA
find_package(CUDA)
set(CUDA_ARCH_BIN "20" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              		
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

#FIND CG Shading
include(cmake/FindCG.cmake)

#FIND PCL
find_package(PCL 1.5 REQUIRED)
#find_package(PCL 1.3 REQUIRED COMPONENTS common visualization io gpu_containers gpu_utils gpu_features)
#find_package(PCL 1.3 REQUIRED)

#FIND OPENCV
set(OpenCV_DIR "C:/Program Files/OpenCV")
find_package(OpenCV REQUIRED)

#FIND OPENGL
find_package(OpenGL REQUIRED)
find_path( GLUT_INCLUDE_DIR GL/glut.h
	${CMAKE_CURRENT_SOURCE_DIR}/externals/freeglut-2.6.0
  )
find_library( GLUT_glut_LIBRARY freeglut_static
	${CMAKE_CURRENT_SOURCE_DIR}/externals/freeglut-2.6.0/lib/win64-vc/
  )
find_path(GLEW_INCLUDE_DIR GL/glew.h
	${CMAKE_CURRENT_SOURCE_DIR}/externals/glew-1.7.0/include
)
find_library(GLEW_LIBRARY glew32s
	${CMAKE_CURRENT_SOURCE_DIR}/externals/glew-1.7.0/lib/win64-vc
)

#FIND GLM
find_path( GLM_INCLUDE_DIR glm/glm.hpp
		PATHS ${CMAKE_CURRENT_SOURCE_DIR}/externals/
	)

SET(Beingthere_PCL_KINFU_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_pcl_kinfu/include)
SET(Beingthere_PCL_CUDA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_pcl_cuda/include)

SET(Beingthere_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_common)	
SET(Beingthere_IO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_io)
SET(Beingthere_GPU_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_gpu)
SET(Beingthere_OPENGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_opengl)
SET(Beingthere_V3D_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_v3d)

add_subdirectory(beingthere_pcl_kinfu)
add_subdirectory(beingthere_pcl_cuda)

add_subdirectory(beingthere_common)
add_subdirectory(beingthere_io)
add_subdirectory(beingthere_gpu)
add_subdirectory(beingthere_opengl)
add_subdirectory(beingthere_v3d)

add_subdirectory(examples)
