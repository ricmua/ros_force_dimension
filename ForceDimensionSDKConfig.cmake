
# Set include directories and library paths for the Force Dimension SDK.

if(UNIX)
  set(ForceDimensionSDK_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")
  set(ForceDimensionSDK_LIBRARIES "${CMAKE_CURRENT_LIST_DIR}/lib/dhdms64.lib")
elseif(WIN32 OR MSVC)
  set(ForceDimensionSDK_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")
  set(FD_DHDMS_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/dhdms.lib")
  set(FD_DHDMS64_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/dhdms64.lib")
  set(ForceDimensionSDK_LIBRARIES "${FD_DHDMS_LIB};${FD_DHDMS64_LIB}")
endif()


