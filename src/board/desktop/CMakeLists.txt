find_package(CURL REQUIRED)
find_package(SDL2 REQUIRED)

add_library(board_desktop STATIC board_gps.c board_render.c)
target_include_directories(board_desktop
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_include_directories(board_desktop PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(board_desktop PRIVATE CURL::libcurl ${SDL2_LIBRARIES})
add_library(board::desktop ALIAS board_desktop)
