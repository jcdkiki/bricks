add_library(physics physics.cpp)
target_link_libraries(physics PUBLIC solvers)
target_include_directories(physics PUBLIC "${CMAKE_SOURCE_DIR}/include/physics")