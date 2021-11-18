cmake_minimum_required(VERSION 3.5.0)
project(choreonoid_ros_practice)

# ビルドタイプをReleaseに設定
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE
    )
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    choreonoid
)

# CMakeのConfigファイルやpkg-configの生成を行わないようにする
# package.xmlでbuild_typeをcmakeにするとConfigファイルがうまく生成されないらしい？
catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)

set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)