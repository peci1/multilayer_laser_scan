cmake_minimum_required(VERSION 2.8.3)
project(multilayer_laser_scan)

set(MESSAGE_DEPS sensor_msgs std_msgs)
set(OTHER_DEPS roscpp)

find_package(catkin REQUIRED COMPONENTS message_generation ${OTHER_DEPS} ${MESSAGE_DEPS})

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES ${MESSAGE_DEPS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime ${OTHER_DEPS} ${MESSAGE_DEPS}
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/scan_iterator.cpp src/MultiLayerLaserScanLayout.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(${CATKIN_ENABLE_TESTING})
  catkin_download_test_data(
    velodyne_hdl_32e.csv
    https://drive.google.com/uc?export=download&id=1lWtIyhL8WK8NnWqUGufIgEp2qtlf-8YQ
    FILENAME velodyne_hdl_32e.csv
    MD5 05f18931e757c6346e7c227722ed1a07
  )
  catkin_download_test_data(
    velodyne_hdl_32e-packets.bag
    https://drive.google.com/uc?export=download&id=1YE5Qwa0NXKrxTYUyx8jtqp7Poh9QC0lZ
    FILENAME velodyne_hdl_32e-packets.bag
    MD5 4a3981ce7e522a8e0f9990dcfeb08b2b
  )
  catkin_download_test_data(
    velodyne_hdl_32e-points.bag
    https://drive.google.com/uc?export=download&id=1lW0u8c34ktfAvpPjks78j5S1Ch_0X0e6
    FILENAME velodyne_hdl_32e-points.bag
    MD5 64b8fce754aeb6514bcd0a1dc431ff0d
  )
  catkin_download_test_data(
    velodyne_hdl_32e-cut_angle-packets.bag
    https://drive.google.com/uc?export=download&id=1EIcZpON6NdS7znrfjpMgjLnAFaxhV24K
    FILENAME velodyne_hdl_32e-cut_angle-packets.bag
    MD5 bc27212dc60de9ec131bf7a2cced4656
  )
  catkin_download_test_data(
    velodyne_hdl_32e-cut_angle-points.bag
    https://drive.google.com/uc?export=download&id=16wUVoxF5eAl5WuksB-LjhN6TG9PU85f5
    FILENAME velodyne_hdl_32e-cut_angle-points.bag
    MD5 8b2f5241841f0f8bc962a071b91761ff
  )
  catkin_download_test_data(
    ouster-os1-64-packets.bag
    https://drive.google.com/uc?export=download&id=1VohD1VBHdxpwuPyNeTbHG1wBMvlnfaeu
    FILENAME ouster-os1-64-packets.bag
    MD5 7ca7c38893531ad79e9071a28535f851
  )
  catkin_download_test_data(
    ouster-os1-64-points.bag
    https://drive.google.com/uc?export=download&id=1XuGcUO4neR-pTGPXxnxl-IONY1wANWIU
    FILENAME ouster-os1-64-points.bag
    MD5 41a38db0afba63e2792fd758db41d2bf
  )

  catkin_add_gtest(scan_layout_test test/scan_layout_test.cpp)
  target_link_libraries(scan_layout_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_compile_definitions(scan_layout_test PRIVATE DATA_DIR="${PROJECT_BINARY_DIR}")

  catkin_add_gtest(scan_iterator_test test/scan_iterator_test.cpp)
  target_link_libraries(scan_iterator_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)