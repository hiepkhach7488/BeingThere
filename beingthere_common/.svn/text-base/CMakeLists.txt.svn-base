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

#Cmake Generate config.h
SET(BEINGTHERE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)
SET(BEINGTHERE_COMMON_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/)
SET(BEINGTHERE_EXAMPLE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../examples/)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/beingthere_config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/beingthere_config.h)

add_definitions(-DFREEGLUT_STATIC)
add_definitions(-DGLEW_STATIC)

SET(LIB_NAME beingthere_common)

FILE(GLOB srcs *.c*)
FILE(GLOB shds_UNC shader/unc/*.vert shader/unc/*.frag shader/unc/*.geom)
FILE(GLOB shds_ETHZ shader/ethz/*.vs shader/ethz/*.cg shader/ethz/*.fs) 
FILE(GLOB shds_OTHERS shader/others/*.vert shader/others/*.frag shader/others/*.geom)

FILE(GLOB incs *.h*)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

source_group("Header Files" FILES incs)
source_group("Header Files\\UNC" FILES ${shds_UNC})
source_group("Header Files\\ETHZ" FILES ${shds_ETHZ})
source_group("Header Files\\OTHERS" FILES ${shds_OTHERS})
source_group("Source Files" FILES srcs)

ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${shds_UNC} ${shds_ETHZ} ${shds_OTHERS})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})

