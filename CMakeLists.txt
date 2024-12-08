cmake_minimum_required(VERSION 3.15)

project(rrt VERSION 1.0 LANGUAGES CXX C)

set(CMAKE_CXX_FLAGS "-g")

find_package(SFML COMPONENTS window graphics system)

add_executable(rrt_viewer src/visual.cpp)
add_executable(performance src/performance.cpp)

target_include_directories(rrt_viewer PUBLIC src)
target_link_libraries(rrt_viewer PUBLIC sfml-window sfml-graphics sfml-system)

target_include_directories(performance PUBLIC src)
target_link_libraries(performance PUBLIC sfml-window sfml-graphics sfml-system)

#################################
# Generate Docs
#################################
find_package(Doxygen)
set(DOXYGEN_OUTPUT_DIRECTORY ../docs)
doxygen_add_docs(docs src)
