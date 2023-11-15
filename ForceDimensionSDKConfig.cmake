
# Set include directories and library paths for the Force Dimension SDK.

if(UNIX)
  set(ForceDimensionSDK_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")
  find_library(FD_LIBDHD 
               NAMES dhd 
               PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/*/*)
               #PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/*
               #PATH_SUFFIXES "lin-x86_64-gcc")
  find_library(FD_LIBDRD NAMES drd PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/*/*)
  set(ForceDimensionSDK_LIBRARIES "${FD_LIBDHD};${FD_LIBDRD}")
elseif(WIN32 OR MSVC)
  set(ForceDimensionSDK_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")
  set(FD_DHDMS_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/dhdms.lib")
  set(FD_DHDMS64_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/dhdms64.lib")
  set(ForceDimensionSDK_LIBRARIES "${FD_DHDMS_LIB};${FD_DHDMS64_LIB}")
endif()


