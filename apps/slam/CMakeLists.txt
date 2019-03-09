set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
# Add apps
add_executable(
  sample_app 
  sample_app.cpp
  # DebugOutput3DWrapper.cpp
  qglViewer3DWrapper.cpp
  ImageReaders.hpp
  ImageReaders.cpp
)

target_include_directories(
  sample_app
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  PRIVATE SYSTEM
  ${QGLVIEWER_INCLUDE_DIR}
)

target_link_libraries(
  sample_app
  lsd_slam
  Qt5::Core
  Qt5::Widgets
  Qt5::Xml
  Qt5::OpenGL
  ${QGLVIEWER_LIBRARY}
) 

set_target_properties(
  sample_app
  PROPERTIES
  CXX_STANDARD 17
)

#add_executable(main_on_images main_on_images.cc DebugOutput3DWrapper.cpp)
#target_link_libraries(main_on_images lsd_slam)

#set_target_properties(main_on_images PROPERTIES CXX_STANDARD 14)
