cmake_minimum_required(VERSION 3.8)
project(limo_manipulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (uses setup.py + setup.cfg)
ament_python_install_package(${PROJECT_NAME})

ament_package()
