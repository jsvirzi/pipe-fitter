cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 14)
add_link_options(-pthread)

#set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD)
# find_package(Threads REQUIRED)
#add_compile_options(-lpthread -lm)

project(pipe-fitter)
add_executable(${PROJECT_NAME} "main.c")
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
