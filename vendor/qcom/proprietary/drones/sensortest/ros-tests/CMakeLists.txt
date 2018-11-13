cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_VERBOSE_MAKEFILE ON)
project(ros_tests)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(${KERNEL_HEADERS})
include_directories(common)
include_directories(server)

add_executable(server server/server.cpp)
target_sources(server PUBLIC common/Publisher.cpp common/IonFdMapperImpl.cpp common/IonFdMapperStub.cpp)
target_link_libraries(server ${catkin_LIBRARIES} ${Boost_LIBRARIES} camera utils cutils pthread binder)
install(TARGETS server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(client client/client.cpp)
target_sources(client PUBLIC common/Publisher.cpp common/IonFdMapperImpl.cpp common/IonFdMapperStub.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${Boost_LIBRARIES} camera utils cutils pthread binder)
install(TARGETS client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/client_server.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  scripts/runtests.py scripts/summarize.py
  PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
