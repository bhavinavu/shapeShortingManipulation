cmake_minimum_required(VERSION 3.5)
project(pymoveit2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(pymoveit2)

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/pick_and_place.py
    ${EXAMPLES_DIR}/throttle_node.py
    ${EXAMPLES_DIR}/color_and_shape_extractor.py
    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
