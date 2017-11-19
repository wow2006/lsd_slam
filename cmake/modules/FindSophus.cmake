set( Sophus_SOURCE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/Sophus/)

set( Sophus_DIR ${Sophus_SOURCE_DIR})

get_filename_component(Sophus_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set( Sophus_INCLUDE_DIR  ${Sophus_DIR} )
set( Sophus_INCLUDE_DIRS ${Sophus_DIR} )
