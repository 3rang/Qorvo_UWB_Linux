file(GLOB QORVO_DRIVER "*.c" "*.h")


find_library(SPI_CORE_LIB libspi_core.so)

# If the library is found, print its path
if(SPI_CORE_LIB)
	message("Found my_library at ${SPI_CORE_LIB}")
else()
    message("my_library not found")
endif()

add_library(Q_DRIVER SHARED ${QORVO_DRIVER})

target_include_directories(Q_DRIVER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../platform/spi_core/inc ${CMAKE_CURRENT_SOURCE_DIR}/../platform/)
target_link_libraries(Q_DRIVER spi_interface)
