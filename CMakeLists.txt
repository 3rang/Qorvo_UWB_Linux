# Set the minimum version of CMake
cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MyProject)

# Set the C standard
set(CMAKE_C_STANDARD 11)

# Add the platform source directory
add_subdirectory(platform)



set(EXAMPLE
    ex_uwb_reading_dev_id
)

foreach(EXAMPLE ${EXAMPLES})
    # Add the library target for this example
    add_subdirectory(example/${EXAMPLES})

    # Link the example library to the main executable
    target_link_libraries(MyExecutable PUBLIC ${EXAMPLES})
endforeach()

# Set the source files for the main executable
set(SOURCES
    main.c
)

# Create the executable for the main program
add_executable(MyExecutable ${SOURCES})

# Link the executable with the spi library
target_link_libraries(MyExecutable PUBLIC spi)

