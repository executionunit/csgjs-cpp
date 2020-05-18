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

set(TEST_CSGJS_SRCS test_csgjscpp.cpp checkimpl.cpp)

add_subdirectory(tp)

add_executable(csgjs ${CSGJS_SRCS})

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


if(CSGJS_TEST_MESHOPTIMIZER)
	target_link_libraries(csgjs meshoptimizer)
	target_compile_definitions(csgjs PRIVATE CSGJS_TEST_MESHOPTIMIZER)
endif()

add_executable(testcsgjs ${TEST_CSGJS_SRCS})
target_link_libraries(testcsgjs doctest::doctest)

if(MSVC)
  target_compile_options(testcsgjs PRIVATE /W4 /WX)
else()
  target_compile_options(testcsgjs PRIVATE -Wall -Wextra -pedantic -Werror)
endif()
