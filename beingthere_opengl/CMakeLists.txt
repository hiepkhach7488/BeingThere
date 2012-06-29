include_directories(
	${OPENGL_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${GLM_INCLUDE_DIR}
	)
link_libraries(
	${OPENGL_LIBRARIES}
	${GLEW_LIBRARY}
	${GLUT_glut_LIBRARY}
)

add_definitions(-DFREEGLUT_STATIC)
add_definitions(-DGLEW_STATIC)

SET(LIB_NAME beingthere_opengl)
FILE(GLOB srcs *.c*)
FILE(GLOB incs *.h*)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

source_group("Header Files" FILES incs)
source_group("Source Files" FILES srcs)

ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})


