cmake_minimum_required(VERSION 3.5)
project(spi C)

set(CMAKE_C_STANDARD 11)

add_library(spi STATIC src/spi.c )
target_include_directories(spi PUBLIC inc)

add_executable(main main.c)
target_link_libraries(main spi)
