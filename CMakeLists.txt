project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)
find_package(roscpp REQUIRED)#测试
add_definitions(-std=c++0x)
INCLUDE_DIRECTORIES("/usr/include/eigen3" src  ${roscpp_INCLUDE_DIRS}) 
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp" "src/*.c" "src/*.cc")
file(GLOB_RECURSE PROJECT_HEADERS "src/*.hpp" "src/*.h" "src/*.hh")

# 设置二进制文件的生成地址，即bin目录，编译后会有名为SensorFusion或SensorFusion_d的二进制文件生成在bin目录下
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 根据RELEASE或DEBUG模式，生成不同名的可执行文件，用以区分
#if( ${CMAKE_BUILD_TYPE} MATCHES "RELEASE" )
add_executable(UnscentedKF ${PROJECT_SOURCES} ${PROJECT_HEADERS})
#add_executable(UnscentedKF ${sources})
target_link_libraries(UnscentedKF ${roscpp_LIBRARIES})#测试
