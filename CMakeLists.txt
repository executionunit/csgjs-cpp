cmake_minimum_required(VERSION 3.14)
set (CMAKE_CXX_STANDARD 11)
project(CSGJSCPP)

option(CSGJS_TEST_MESHOPTIMIZER "Test Mesh Optimizer" OFF)

set(CSGJS_SRCS
    main.cpp
	checkimpl.cpp
    csgjs.h
	mycsgjs.h
)

add_subdirectory(tp)

add_executable(csgjs ${CSGJS_SRCS})

target_link_libraries(csgjs meshoptimizer)
if(CSGJS_TEST_MESHOPTIMIZER)
	target_compile_definitions(csgjs PRIVATE CSGJS_TEST_MESHOPTIMIZER)
endif()
if(MSVC)
  target_compile_options(csgjs PRIVATE /W4 /WX)
else()
  target_compile_options(csgjs PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

add_custom_target (MiscFiles SOURCES
    README.md
    LICENSE
	.clang-format
	.clang-tidy
	.gitignore
)