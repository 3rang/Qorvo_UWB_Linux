# Set the source files
set(SOURCES
    read_device_id.c
)

# Create the executable
add_executable(read_device_id ${SOURCES})

# Link the executable with the spi library
target_link_libraries(read_device_id PRIVATE spi)

