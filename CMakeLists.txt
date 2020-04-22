cmake_minimum_required(VERSION 3.16)

project(CSGJSCPP)

set(CSGJS_SRCS
    main.cpp
    csgjs.h
)

add_executable(csgjs ${CSGJS_SRCS})

add_custom_target (MiscFiles SOURCES
    README.md
    LICENSE
	.clang-format
	.clang-tidy
	.gitignore
)