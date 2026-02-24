cmake_minimum_required(VERSION 3.8)
project(rh56dfx_description)

find_package(ament_cmake REQUIRED)

# --------------------------------------------------
# Install URDF/Xacro files
# --------------------------------------------------
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Install Meshes (recursive, keeps folder structure)
# --------------------------------------------------
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Install Launch files
# --------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Install RViz configs
# --------------------------------------------------
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
