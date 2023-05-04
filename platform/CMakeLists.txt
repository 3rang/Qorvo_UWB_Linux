# Add the SPI core directory
add_subdirectory(spi_core)

# Set the platform source files
set(SOURCES
    spi_interface.c
)

# Add the platform library target
add_library(spi_interface SHARED ${SOURCES})

#include_directories(${CMAKE_CURRENT_INCLUDE_DIR}/spi_core/inc/)

# Link the platform library with the SPI core library
target_link_libraries(spi_interface PUBLIC spi_core)

# Include the platform header files
target_include_directories(spi_interface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/spi_core/inc ${CMAKE_CURRENT_SOURCE_DIR} )
