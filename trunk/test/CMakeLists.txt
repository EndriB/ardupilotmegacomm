include_directories ( "${PROJECT_SOURCE_DIR}/src")

add_executable(bincomm bincomm.cpp)
target_link_libraries(bincomm apmcomm)

add_executable(hilcomm hilcomm.cpp)
target_link_libraries(hilcomm apmcomm)
