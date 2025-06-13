set(CMAKE_C_EXTENSIONS ON)

find_package(CURL REQUIRED)
find_package(SDL2 REQUIRED)

add_library(
  board_linux OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/board.c ${CMAKE_CURRENT_SOURCE_DIR}/board_gps.c
  ${CMAKE_CURRENT_SOURCE_DIR}/board_render.c
  ${CMAKE_CURRENT_SOURCE_DIR}/freertos_hooks.c)
target_include_directories(board_linux
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# configure GPS and rendering backend
target_include_directories(board_linux PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(board_linux PRIVATE CURL::libcurl ${SDL2_LIBRARIES})

# configure FreeRTOS
add_subdirectory(freertos)

set(FREERTOS_HEAP
    "3"
    CACHE STRING "" FORCE)
set(FREERTOS_PORT
    "GCC_POSIX"
    CACHE STRING "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/ext/FreeRTOS
                 ${CMAKE_CURRENT_BINARY_DIR}/ext/FreeRTOS)

target_link_libraries(board_linux PUBLIC freertos_config freertos_kernel)

# export alias
add_library(board::linux ALIAS board_linux)
