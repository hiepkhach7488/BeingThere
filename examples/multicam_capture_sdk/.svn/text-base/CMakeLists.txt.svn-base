GET_FILENAME_COMPONENT(folderName ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(APP_NAME "${folderName}")

FILE(GLOB header *.h*)
FILE(GLOB source *.cpp)
FILE(GLOB shader */*.vert */*.geom */*.frag */*.cg */*.vs */*.fs)
  
source_group("Shader Files" FILES ${shader})

SET(LIST_INCLUDE_DIR ${LIST_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${Beingthere_OPENGL_INCLUDE_DIR} ${GLM_INCLUDE_DIR})
SET(LIST_LIBRARIES  ${LIST_LIBRARIES}	${OPENGL_LIBRARIES} ${GLEW_LIBRARY} ${GLUT_glut_LIBRARY} beingthere_opengl)
add_definitions(-DFREEGLUT_STATIC)
add_definitions(-DGLEW_STATIC)

FIND_PATH(KINECT_SDK_INCLUDE_PATH NuiSensor.h
	$ENV{KINECTSDK10_DIR}/inc/
  )
  
FIND_LIBRARY(KINECT_SDK_LIBRARY
  NAMES Kinect10.lib
  PATHS $ENV{KINECTSDK10_DIR}/lib/amd64/
  
)
	  
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLM_INCLUDE_DIR}
    ${Beingthere_OPENGL_INCLUDE_DIR}
    ${CG_INCLUDE_PATH}
    ${Beingthere_V3D_INCLUDE_DIR}
	${LIST_INCLUDE_DIR}
	${KINECT_SDK_INCLUDE_PATH}
)
  

add_definitions(-DV3DLIB_ENABLE_GPGPU)
add_definitions(-DV3DLIB_GPGPU_ENABLE_CG)
  
set(the_target ${APP_NAME})

CUDA_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${header} ${source} ${shader})
  
target_link_libraries(${the_target} ${OpenCV_LIBS} beingthere_opengl ${CG_LIBRARY} ${CG_GL_LIBRARY} beingthere_v3d ${OPENNI_LIBRARY} ${LIST_LIBRARIES} ${KINECT_SDK_LIBRARY})