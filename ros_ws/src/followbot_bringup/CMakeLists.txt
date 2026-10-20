cmake_minimum_required(VERSION 3.8)
project(followbot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# launch ファイルから import する純関数を置く (followbot_bringup.launch_args)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config maps behavior_trees
        DESTINATION share/${PROJECT_NAME})

ament_package()
