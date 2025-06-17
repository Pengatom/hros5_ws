cmake_minimum_required(VERSION 3.10)
project(hros5_demos)

find_package(ament_cmake REQUIRED)

# Install launch, config, and description directories
install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

# (Optional) If you have any Python scripts in a scripts/ folder:
# install(PROGRAMS
#   scripts/your_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
