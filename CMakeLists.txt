# Set the minimum version of CMake
cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MyProject)

# Set the C standard
set(CMAKE_C_STANDARD 11)

# Add the platform source directory
add_subdirectory(platform)
include_directories(platform)


if(NOT DEFINED EXAMPLE)
    set(EXAMPLE "ex_uwb_reading_dev_id" CACHE STRING "Name of the example subfolder to include")
    message("Selected folder: ${EXAMPLE}")
endif()

#add_subdirectory("example/${EXAMPLE}")


# Set the source files for the main executable
set(SOURCES
    main.c
    example/${EXAMPLE}/read_device_id.c
)

# Create the executable for the main program
add_executable(MyExecutable ${SOURCES} )

# Link the executable with the spi library
target_link_libraries(MyExecutable PUBLIC spi_core)
