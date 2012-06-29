MACRO(LIST_CONTAINS var value)
  SET(${var} FALSE)
  FOREACH (value2 ${ARGN})
    IF (${value} STREQUAL ${value2})
      SET(${var} TRUE)
    ENDIF (${value} STREQUAL ${value2})
  ENDFOREACH (value2)
ENDMACRO(LIST_CONTAINS)


MACRO (MACRO_EXAMPLE_APPLICATION)

# IF(${ARGC} GREATER 0)
	# SET(APP_NAME "${ARGV0}")
# ELSE(${ARGC} GREATER 0)
	GET_FILENAME_COMPONENT(folderName ${CMAKE_CURRENT_SOURCE_DIR} NAME)
	SET(APP_NAME "${folderName}")
# ENDIF(${ARGC} GREATER 0)

FILE(GLOB header *.h*)
FILE(GLOB source *.cpp)
FILE(GLOB shader */*.vert */*.geom */*.frag */*.cg */*.vs */*.fs)

source_group("Shader Files" FILES ${shader})

SET(ARGS
	${ARGV1}
	${ARGV2}
	${ARGV3}
	${ARGV4}
	${ARGV5}
	${ARGV6}
	${ARGV7}
	${ARGV8}
)

LIST_CONTAINS(contains OPENGL ${ARGS})
IF (contains)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${Beingthere_OPENGL_INCLUDE_DIR} ${GLM_INCLUDE_DIR})
  SET(LIST_LIBRARIES  ${LIST_LIBRARIES}	${OPENGL_LIBRARIES} ${GLEW_LIBRARY} ${GLUT_glut_LIBRARY} beingthere_opengl)
  add_definitions(-DFREEGLUT_STATIC)
  add_definitions(-DGLEW_STATIC)
ENDIF (contains)

LIST_CONTAINS(contains OPENCV ${ARGS})
IF (contains)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
  SET(LIST_LIBRARIES ${LIST_LIBRARIES} ${OpenCV_LIBS} )
ENDIF (contains)

LIST_CONTAINS(contains CG ${ARGS})
IF (contains)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${CG_INCLUDE_PATH})
  SET(LIST_LIBRARIES ${LIST_LIBRARIES} ${CG_LIBRARY} ${CG_GL_LIBRARY})
ENDIF (contains)

LIST_CONTAINS(contains V3D ${ARGS})
IF (contains)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${Beingthere_V3D_INCLUDE_DIR})
  SET(LIST_LIBRARIES ${LIST_LIBRARIES} beingthere_v3d)
  add_definitions(-DV3DLIB_ENABLE_GPGPU)
  add_definitions(-DV3DLIB_GPGPU_ENABLE_CG)
ENDIF (contains)

LIST_CONTAINS(contains_cuda CUDA ${ARGS})
LIST_CONTAINS(contains_pcl PCL ${ARGS})

IF(contains_pcl)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
  
  link_directories(${PCL_LIBRARY_DIRS})
  SET(LIST_LIBRARIES ${LIST_LIBRARIES}  ${OPENNI_LIBRARY} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
					 ${PCL_VISUALIZATION_LIBRARIES} ${PCL_KDTREE_LIBRARIES})
					 
  add_definitions(${PCL_DEFINITIONS})

ENDIF(contains_pcl)

IF(contains_cuda)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${Beingthere_GPU_INCLUDE_DIR})
  SET(LIST_LIBRARIES ${LIST_LIBRARIES}  beingthere_gpu)
ENDIF(contains_cuda)

IF (contains_pcl AND contains_cuda)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${Beingthere_PCL_CUDA_INCLUDE_DIR})
  SET(LIST_LIBRARIES ${LIST_LIBRARIES}  beingthere_pcl_cuda ${PCL_CUDA_IO_LIBRARIES})
ENDIF (contains_pcl AND contains_cuda)

LIST_CONTAINS(contains BEING_THERE ${ARGS})
IF (contains)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${Beingthere_IO_INCLUDE_DIR}  )
  SET(LIST_LIBRARIES ${LIST_LIBRARIES} beingthere_io)
ENDIF (contains)

LIST_CONTAINS(contains PCL_KINFU ${ARGS})
IF (contains)
  SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${Beingthere_PCL_KINFU_INCLUDE_DIR}  )
  SET(LIST_LIBRARIES ${LIST_LIBRARIES} beingthere_pcl_kinfu)
ENDIF (contains)

include_directories(
    ${LIST_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${Beingthere_COMMON_INCLUDE_DIR}
)

SET(the_target ${APP_NAME})
IF(contains_cuda)
	CUDA_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${header} ${source} ${shader})
ELSE(contains_cuda)
	ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${header} ${source} ${shader})
ENDIF(contains_cuda)

LIST_CONTAINS(contains PRINT_OUT ${ARGS})
IF (contains)
	message("LIST_INCLUDES: " ${LIST_INCLUDE_DIR})
ENDIF (contains)

target_link_libraries(${the_target} ${LIST_LIBRARIES}  beingthere_common)  

ENDMACRO (MACRO_EXAMPLE_APPLICATION)
