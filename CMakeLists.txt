# Set the minimum version of CMake
cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MyProject)

# Set the C standard
set(CMAKE_C_STANDARD 11)

# Add the platform source directory
include_directories(platform)
include_directories(uwb_driver)

add_subdirectory(platform)
add_subdirectory(uwb_driver)


if(NOT DEFINED EXAMPLE)
	#set(EXAMPLE "ex_uwb_reading_dev_id" CACHE STRING "Name of the example subfolder to include")
    set(EXAMPLE "ex_uwb_reading_dev" CACHE STRING "Name of the example subfolder to include")
    message("Selected folder: ${EXAMPLE}")
endif()


# Set the source files for the main executable
set(SOURCES
    main.c
    #example/${EXAMPLE}/read_device_id.c
    example/${EXAMPLE}/ex_uwb_reading_dev.c
)

# Create the executable for the main program
add_executable(MyExecutable ${SOURCES})

# Link the executable with the spi library
target_link_libraries(MyExecutable PUBLIC Q_DRIVER)
