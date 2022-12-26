cmake_minimum_required(VERSION 3.8)
project(fast_astar)

add_library(fast_astar SHARED AStar.c AStar.h)
target_link_libraries(fast_astar m)
#target_include_directories(fast_astar PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories(fast_astar PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
# Set the public header property to the one with the actual API.
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER AStar.h)