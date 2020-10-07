cmake_minimum_required(VERSION 2.8.12)
project(ivcommon)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_ivcommon_project()
#google_enable_testing()
find_package(Eigen3 REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
#find_package(OpenCV REQUIRED)
#find_package (Glog REQUIRED)
#find_package(LuaGoogle REQUIRED)


find_package(Protobuf REQUIRED)

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)
install(FILES package.xml DESTINATION share/ivcommon)


install(DIRECTORY cmake DESTINATION share/ivcommon/)

file(GLOB_RECURSE ALL_COMMON_SRCS "ivcommon/io/*.cc" "ivcommon/io/*.h" "ivcommon/common/*.cc" "ivcommon/common/*.h"
 "ivcommon/sensor/*.cc" "ivcommon/sensor/*.h" "ivcommon/transform/*.cc" "ivcommon/transform/*.h"
 "ivcommon/transform/utm/*.cc" "ivcommon/transform/utm/*.h")



file(GLOB_RECURSE ALL_UTIL_SRCS "ivcommon/util/*")

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

foreach(ABS_FIL ${ALL_PROTOS})

  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    PRE_BUILD
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_COMMON_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})




set(LINKER_LIBS "")



include("cmake/FindGlog.cmake")

list(APPEND LINKER_LIBS ${GLOG_LIBRARIES})

include("cmake/FindGFlags.cmake")

list(APPEND LINKER_LIBS ${GFLAGS_LIBRARIES})

include("cmake/FindLuaGoogle.cmake")

list(APPEND LINKER_LIBS ${LUA_LIBRARIES})


list(APPEND LINKER_LIBS ${PROTOBUF_LIBRARY})






#add_library(utils SHARED  ${ALL_UTIL_SRCS})
#target_link_libraries(utils  PUBLIC  ${OpenCV_LIBS} ${LINKER_LIBS} )
add_library(ivcommon SHARED ${ALL_COMMON_SRCS})

target_link_libraries(ivcommon  PUBLIC  ${LINKER_LIBS}  ${EIGEN3_LIBRARIES})


target_include_directories(ivcommon  SYSTEM  PUBLIC  "${GLOG_INCLUDE_DIRS}")
target_include_directories(ivcommon  SYSTEM  PUBLIC  "${GFLAGS_INCLUDE_DIRS}")
target_include_directories(ivcommon  SYSTEM  PUBLIC  "${LUA_INCLUDE_DIR}")
target_include_directories(ivcommon  SYSTEM  PUBLIC  "${EIGEN3_INCLUDE_DIR}")


target_include_directories(ivcommon PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/ivcommon>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/ivcommon>
    $<INSTALL_INTERFACE:include/ivcommon>
)




target_include_directories(ivcommon PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)




set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11 ${GOOG_CXX_FLAGS}")
set_target_properties(ivcommon PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(
  TARGETS ivcommon
  EXPORT ivcommonExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install source headers.
file(GLOB_RECURSE hdrs "*.h" "*.proto")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)

  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# Install generated headers.
file(GLOB_RECURSE hdrs "*.h.in")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  install(
    FILES
      ${PROJECT_BINARY_DIR}/${DIR}/${FILE_WE}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)

  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

set(IVCOMMON_CMAKE_DIR share/ivcommon/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ivcommon-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/ivcommon/ivcommon-config.cmake
  PATH_VARS IVCOMMON_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ivcommon
)

install(
  EXPORT ivcommonExport
  DESTINATION share/ivcommon/cmake/
  FILE IVCOMMONTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/ivcommon/ivcommon-config.cmake
  DESTINATION share/ivcommon/
)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
