cmake_minimum_required(VERSION 3.0.0)
project(Eloy VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    message("Lustrine: using gcc")
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined")
    else()
        set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3 -mavx2 -ffast-math  -mfma -march=skylake")
    endif()
endif()


set(LEVEKGL_BUILD_SAMPLES OFF CACHE BOOL "Do not build LevekGL samples" FORCE)

add_subdirectory(thirdparty)
link_directories(${Eloy_SOURCE_DIR}/source)

set(ELOY_SOURCES
    "source/Eloy.cpp"
    "source/Engine.cpp"
    "source/profiling/tsc_x86.cpp"
)

add_library(Eloy STATIC ${ELOY_SOURCES})

set(ELOY_TARGET_INCLUDE_DIRS
    PUBLIC "source"
    PUBLIC "thirdparty/LevekGL"   
)

target_include_directories(Eloy ${ELOY_TARGET_INCLUDE_DIRS})

target_link_libraries(Eloy LevekGL)

add_compile_definitions(ELOY_SOURCE_DIRECTORY="${Eloy_SOURCE_DIR}/source")

add_subdirectory(samples)

