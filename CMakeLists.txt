cmake_minimum_required(VERSION 3.16)
project(sparkmax C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(Threads REQUIRED)
set(WARN -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)

set(SM_SOURCES src/sparkmax.c src/slcan.c)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  list(APPEND SM_SOURCES src/socketcan.c)
endif()

add_library(sparkmax STATIC ${SM_SOURCES})
target_include_directories(sparkmax PUBLIC include)
target_compile_options(sparkmax PRIVATE ${WARN})
target_link_libraries(sparkmax PUBLIC Threads::Threads)

add_executable(sm_sniff tools/sm_sniff.cpp)
target_compile_options(sm_sniff PRIVATE -Wall -Wextra -Werror)
target_link_libraries(sm_sniff PRIVATE sparkmax)

add_executable(sm_drive examples/sm_drive.c)
target_compile_options(sm_drive PRIVATE ${WARN})
target_link_libraries(sm_drive PRIVATE sparkmax)

enable_testing()
add_executable(test_codec tests/test_codec.c)
target_compile_options(test_codec PRIVATE -Wall -Wextra -Werror)
target_link_libraries(test_codec PRIVATE sparkmax m)
add_test(NAME codec COMMAND test_codec)
