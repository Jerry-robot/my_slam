find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRE})
list(APPEND ALL_TARGET_LIBRARIES ${PCL_LIBRARIES})