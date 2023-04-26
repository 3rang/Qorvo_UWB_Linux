# Add the source files for the platform library
set(SOURCES
    src/spi.c
)

# Create the platform library target
add_library(spi SHARED ${SOURCES})

# Set the include directories for the platform library
target_include_directories(spi PUBLIC inc)
