cmake_minimum_required (VERSION 2.6)
project (ardupilotmega)

# The version number.
set(ardupilotmega_VERSION_MAJOR 0)
set(ardupilotmega_VERSION_MINOR 0)
set(ardupilotmega_VERSION_PATCH 0)
set(ardupilotmega_SOVERSION 0)

set(ardupilotmega_VERISON ${ardupilotmega_VERSION_MAJOR}.${ardupilotmega_VERSION_MINOR}.${ardupilotmega_VERSION_PATCH})

# installer
include (InstallRequiredSystemLibraries)
set (CPACK_PACKAGE_CONTACT "James Goppert james.goppert@gmail.com")
set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
set (CPACK_PACKAGE_VERSION_MAJOR "${ardupilotmega_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${ardupilotmega_VERSION_MINOR}")
set (CPACK_PACKAGE_VERSION_PATCH "${ardupilotmega_VERSION_PATCH}")
include (CPack)

# boost
find_package(Boost 1.40 REQUIRED COMPONENTS thread-mt system-mt)
link_directories( ${Boost_LIBRARIES_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

# subdirectories
add_subdirectory(src)
add_subdirectory(test)
