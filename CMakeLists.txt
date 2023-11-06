cmake_minimum_required(VERSION 3.5)
project(transform_mapping_polygon)
set(CMAKE_CXX_STANDARD 17)


set(COMPILE_TESTS 1)
set(COMPILE_DEMOS 1)
set(OpenCV_DIR /home/vitaly/opencv/install_4_5_2_contrib_cuda)
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)

file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/*.hpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS}) # этот инклюд нужно ставить строго после find_package(OpenCV ...

add_library(transform_mappinp_polygon SHARED ${SOURCES} ${HEADERS}) # ${HEADERS} не обязательно, тк нужно только для отладки

# add_executable(test_convex_polygon tests/test_convex_polygon.cpp)
# target_link_libraries(test_convex_polygon ${OpenCV_LIBS} transform_mappinp_polygon)

# add_executable(test_equation_find_pos_map tests/test_equation_find_pos_map.cpp)
# target_link_libraries(test_equation_find_pos_map ${OpenCV_LIBS} transform_mappinp_polygon)
    
# add_executable(test_mesh_triangle tests/test_mesh_triangle.cpp)
# target_link_libraries(test_mesh_triangle ${OpenCV_LIBS} transform_mappinp_polygon)

# add_executable(test_triangle_warp tests/test_triangle_warp.cpp)
# target_link_libraries(test_triangle_warp ${OpenCV_LIBS} transform_mappinp_polygon)

# add_executable(test_get_perspective_transform_mat tests/test_get_perspective_transform_mat.cpp)
# target_link_libraries(test_get_perspective_transform_mat ${OpenCV_LIBS} transform_mappinp_polygon)

if(${COMPILE_TESTS} EQUAL 1)
    file(GLOB tests_src "${CMAKE_CURRENT_LIST_DIR}/tests/*.cpp")
    foreach(test_src ${tests_src})
        get_filename_component(test_NAME "${test_src}" NAME_WE)
        #set(test_NAME test_${test_NAME})
        add_executable(${test_NAME} ${test_src})
        target_link_libraries(${test_NAME} transform_mappinp_polygon ${OpenCV_LIBS})
    endforeach()
endif()

if(${COMPILE_DEMOS} EQUAL 1)
    file(GLOB demo_src "${CMAKE_CURRENT_LIST_DIR}/demo/*.cpp")
    foreach(demo_src ${demo_src})
        get_filename_component(demo_NAME "${demo_src}" NAME_WE)
        #set(test_NAME test_${test_NAME})
        add_executable("demo_${demo_NAME}" ${demo_src})
        target_link_libraries("demo_${demo_NAME}" transform_mappinp_polygon ${OpenCV_LIBS})
    endforeach()
endif()

