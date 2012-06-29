# - MACRO_OPTIONAL_ADD_SUBDIRECTORY() combines ADD_SUBDIRECTORY() with an OPTION()
# MACRO_OPTIONAL_ADD_SUBDIRECTORY( <dir> )
# If you use MACRO_OPTIONAL_ADD_SUBDIRECTORY() instead of ADD_SUBDIRECTORY(),
# this will have two effects
# 1 - CMake will not complain if the directory doesn't exist
#     This makes sense if you want to distribute just one of the subdirs
#     in a source package, e.g. just one of the subdirs in kdeextragear.
# 2 - If the directory exists, it will offer an option to skip the 
#     subdirectory.
#     This is useful if you want to compile only a subset of all
#     directories.

# Copyright (c) 2007, Alexander Neundorf, <neundorf@kde.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


MACRO (MACRO_OPTIONAL_ADD_SUBDIRECTORY _dir) # plus optional argument specifying if build should be skipped

   GET_FILENAME_COMPONENT(_fullPath ${_dir} ABSOLUTE)
   GET_FILENAME_COMPONENT(_relFolder ${CMAKE_CURRENT_SOURCE_DIR} NAME)
   IF(EXISTS ${_fullPath})
      SET(_DEFAULT_OPTION_VALUE TRUE)
      IF(DISABLE_ALL_OPTIONAL_SUBDIRS  AND NOT DEFINED  OPENTL_BUILD_${_relFolder}_${_dir})
         SET(_DEFAULT_OPTION_VALUE FALSE)
      ENDIF(DISABLE_ALL_OPTIONAL_SUBDIRS  AND NOT DEFINED  OPENTL_BUILD_${_relFolder}_${_dir})
      OPTION(OPENTL_BUILD_${_relFolder}_${_dir} "Build directory ${_dir}" ${_DEFAULT_OPTION_VALUE})
      MARK_AS_ADVANCED(OPENTL_BUILD_${_relFolder}_${_dir})

      IF(${ARGC} GREATER 1)
          SET(BUILD_TARGET ${ARGV1})
      ELSE(${ARGC} GREATER 1)
          SET(BUILD_TARGET TRUE)
      ENDIF(${ARGC} GREATER 1)

      IF(BUILD_TARGET)
           IF(OPENTL_BUILD_${_relFolder}_${_dir})
             ADD_SUBDIRECTORY(${_dir})
          ELSE(OPENTL_BUILD_${_relFolder}_${_dir})
             MESSAGE(STATUS "SKIPPED: ${_dir} (for this build by CMakeCache.txt)")
          ENDIF(OPENTL_BUILD_${_relFolder}_${_dir})
      ELSE(BUILD_TARGET)
         MESSAGE(STATUS "SKIPPED: ${_dir} (permanently by CMakeLists.txt)")
     ENDIF(BUILD_TARGET)
   ELSE(EXISTS ${_fullPath})
	MESSAGE(STATUS "SKIPPED: ${_dir} (directory not found)")
   ENDIF(EXISTS ${_fullPath})
ENDMACRO (MACRO_OPTIONAL_ADD_SUBDIRECTORY)
