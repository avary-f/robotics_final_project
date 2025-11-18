# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(chess_master_CONFIG_INCLUDED)
  return()
endif()
set(chess_master_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(chess_master_SOURCE_PREFIX /home/radlab/Documents/robotics_final_project/src/chess_master)
  set(chess_master_DEVEL_PREFIX /home/radlab/Documents/robotics_final_project/devel)
  set(chess_master_INSTALL_PREFIX "")
  set(chess_master_PREFIX ${chess_master_DEVEL_PREFIX})
else()
  set(chess_master_SOURCE_PREFIX "")
  set(chess_master_DEVEL_PREFIX "")
  set(chess_master_INSTALL_PREFIX /home/radlab/Documents/robotics_final_project/install)
  set(chess_master_PREFIX ${chess_master_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'chess_master' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(chess_master_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(chess_master_INCLUDE_DIRS "")
  set(_include_dirs "")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'radlab <radlab@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${chess_master_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'chess_master' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'chess_master' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/radlab/Documents/robotics_final_project/src/chess_master/${idir}'.  ${_report}")
    endif()
    _list_append_unique(chess_master_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND chess_master_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND chess_master_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT chess_master_NUM_DUMMY_TARGETS)
      set(chess_master_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::chess_master::wrapped-linker-option${chess_master_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR chess_master_NUM_DUMMY_TARGETS "${chess_master_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::chess_master::wrapped-linker-option${chess_master_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND chess_master_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND chess_master_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND chess_master_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/radlab/Documents/robotics_final_project/devel/lib;/home/radlab/Documents/robotics_final_project/devel/lib;/home/radlab/Desktop/robotics_ws/devel/lib;/home/radlab/baxter_ws/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(chess_master_LIBRARY_DIRS ${lib_path})
      list(APPEND chess_master_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'chess_master'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND chess_master_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(chess_master_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${chess_master_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "rospy;std_msgs;sensor_msgs;geometry_msgs;tf2_ros;baxter_interface")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 chess_master_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${chess_master_dep}_FOUND)
      find_package(${chess_master_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${chess_master_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(chess_master_INCLUDE_DIRS ${${chess_master_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(chess_master_LIBRARIES ${chess_master_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${chess_master_dep}_LIBRARIES})
  _list_append_deduplicate(chess_master_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(chess_master_LIBRARIES ${chess_master_LIBRARIES})

  _list_append_unique(chess_master_LIBRARY_DIRS ${${chess_master_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(chess_master_EXPORTED_TARGETS ${${chess_master_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${chess_master_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
