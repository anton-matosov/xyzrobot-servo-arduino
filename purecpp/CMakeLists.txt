cmake_minimum_required(VERSION 3.11)

project(Servo CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


file(GLOB FILES *.cxx *.cpp *.h *.hpp)

add_executable(Servo ${FILES})

find_package(Boost REQUIRED system)
target_link_libraries(Servo PRIVATE Boost::boost ${Boost_LIBRARIES})
