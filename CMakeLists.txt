cmake_minimum_required(VERSION 3.0.0)
project(Eloy VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

set(LEVEKGL_BUILD_SAMPLES OFF CACHE BOOL "Do not build LevekGL samples" FORCE)

add_subdirectory(thirdparty)
link_directories(${Eloy_SOURCE_DIR}/source)

set(ELOY_SOURCES
    "source/Eloy.cpp"
    "source/Engine.cpp"
)

add_library(Eloy SHARED ${ELOY_SOURCES})

set(ELOY_TARGET_INCLUDE_DIRS
    PUBLIC "source"
    PUBLIC "thirdparty/LevekGL"   
)

target_include_directories(Eloy ${ELOY_TARGET_INCLUDE_DIRS})

target_link_libraries(Eloy LevekGL)

add_subdirectory(samples)

