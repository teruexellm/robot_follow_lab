cmake_minimum_required(VERSION 3.8)
project(followbot_description)

find_package(ament_cmake REQUIRED)

# launch は含めない。このパッケージに launch は無く、空ディレクトリを
# 指していた。git は空ディレクトリを追跡しないため、clone 先で
#   ament_cmake_symlink_install_directory() can't find '.../launch'
# となりビルドが落ちていた (DEVLOG B-4)。launch は followbot_bringup にある。
install(DIRECTORY urdf config
        DESTINATION share/${PROJECT_NAME})

ament_package()
