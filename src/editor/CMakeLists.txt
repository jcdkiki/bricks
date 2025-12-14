add_executable(editor
    main.cpp
    render.cpp
    glad.c
)
target_link_libraries(editor PRIVATE glfw imgui physics)
target_include_directories(editor PRIVATE "${CMAKE_SOURCE_DIR}/include")
