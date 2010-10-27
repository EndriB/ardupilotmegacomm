cmake_minimum_required (VERSION 2.6)
project (apmcomm)

# The version number.
set(apmcomm_VERSION_MAJOR 0)
set(apmcomm_VERSION_MINOR 0)
set(apmcomm_VERSION_PATCH 0)
set(apmcomm_SOVERSION 0)

set(apmcomm_VERISON ${apmcomm_VERSION_MAJOR}.${apmcomm_VERSION_MINOR}.${apmcomm_VERSION_PATCH})

# installer
include (InstallRequiredSystemLibraries)
set (CPACK_PACKAGE_CONTACT "James Goppert james.goppert@gmail.com")
set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
set (CPACK_PACKAGE_VERSION_MAJOR "${apmcomm_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${apmcomm_VERSION_MINOR}")
set (CPACK_PACKAGE_VERSION_PATCH "${apmcomm_VERSION_PATCH}")
include (CPack)

# boost
find_package(Boost 1.40 REQUIRED COMPONENTS thread-mt system-mt)
link_directories( ${Boost_LIBRARIES_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

# subdirectories
add_subdirectory(src)
add_subdirectory(test)
