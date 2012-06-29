include_directories(
	${OPENGL_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${GLM_INCLUDE_DIR}
	${CG_INCLUDE_PATH}
	)
link_libraries(
	${OPENGL_LIBRARIES}
	${GLEW_LIBRARY}
	${GLUT_glut_LIBRARY}
)

add_definitions(-DFREEGLUT_STATIC)
add_definitions(-DGLEW_STATIC)
add_definitions(-DV3DLIB_ENABLE_GPGPU)
add_definitions(-DV3DLIB_GPGPU_ENABLE_CG)

SET(LIB_NAME beingthere_v3d)
FILE(GLOB srcs */*.c*)
FILE(GLOB incs */*.h*)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

source_group("Header Files" FILES incs)
source_group("Source Files" FILES srcs)

ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${CG_LIBRARY} ${CG_GL_LIBRARY})

