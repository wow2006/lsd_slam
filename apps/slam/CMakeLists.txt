# Add apps
add_executable(sample_app sample_app.cc DebugOutput3DWrapper.cpp)
target_link_libraries(sample_app lsd_slam)

set_target_properties(sample_app PROPERTIES CXX_STANDARD 14)

add_executable(main_on_images main_on_images.cc DebugOutput3DWrapper.cpp)
target_link_libraries(main_on_images lsd_slam)

set_target_properties(main_on_images PROPERTIES CXX_STANDARD 14)
