if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	add_definitions(-DHAVE_OPENCV)	
	link_directories(${OpenCV_LIB_DIR})
else()
	remove_definitions(-DHAVE_OPENCV)
endif()

SET(LIB_NAME beingthere_io)
FILE(GLOB srcs *.c*)
FILE(GLOB incs *h.*)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS})

source_group("Header Files" FILES incs)
source_group("Source Files" FILES srcs)

ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${BOOST_LIBRARIES} ${OPENNI_LIBRARY})

