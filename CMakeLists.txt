cmake_minimum_required(VERSION 3.0.2)
project(mobipick_api)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package()
