cmake_minimum_required(VERSION 3.16)
set (CMAKE_CXX_STANDARD 11)
project(CSGJSCPP)

set(CSGJS_SRCS
    main.cpp
	checkimpl.cpp
    csgjs.h
)

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