# Add the SPI core directory
add_subdirectory(spi_core)

# Set the platform source files
set(SOURCES
    spi_interface.c
)

# Add the platform library target
add_library(platform ${SOURCES})

# Link the platform library with the SPI core library
target_link_libraries(platform PUBLIC spi_core)

# Include the platform header files
target_include_directories(platform PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

