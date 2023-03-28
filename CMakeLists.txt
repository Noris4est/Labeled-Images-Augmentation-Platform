cmake_minimum_required(VERSION 3.5)
project(transform_mapping_polygon)
set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR /home/vitaly/opencv/install_4_5_2_contrib)
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)

file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/*.hpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS}) # этот инклюд нужно ставить строго после find_package(OpenCV ...

add_library(transform_mappinp_polygon SHARED ${SOURCES} ${HEADERS}) # ${HEADERS} не обязательно, тк нужно только для отладки

add_executable(test_convex_polygon tests/test_convex_polygon.cpp)
target_link_libraries(test_convex_polygon ${OpenCV_LIBS} transform_mappinp_polygon)

add_executable(test_equation_find_pos_map tests/test_equation_find_pos_map.cpp)
target_link_libraries(test_equation_find_pos_map ${OpenCV_LIBS} transform_mappinp_polygon)
    
add_executable(test_mesh_triangle tests/test_mesh_triangle.cpp)
target_link_libraries(test_mesh_triangle ${OpenCV_LIBS} transform_mappinp_polygon)

add_executable(test_triangle_warp tests/test_triangle_warp.cpp)
target_link_libraries(test_triangle_warp ${OpenCV_LIBS} transform_mappinp_polygon)
