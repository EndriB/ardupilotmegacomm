cmake_minimum_required (VERSION 2.6)
project (ardupilotmegacomm)

# The version number.
set(ardupilotmegacomm_VERSION_MAJOR 0)
set(ardupilotmegacomm_VERSION_MINOR 0)
set(ardupilotmegacomm_VERSION_PATCH 1)
set(ardupilotmegacomm_SOVERSION 0)

set(ardupilotmegacomm_VERISON ${ardupilotmegacomm_VERSION_MAJOR}.${ardupilotmegacomm_VERSION_MINOR}.${ardupilotmegacomm_VERSION_PATCH})

# installer
include (InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_CONTACT "James Goppert james.goppert@gmail.com")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
set(CPACK_PACKAGE_VERSION_MAJOR "${ardupilotmegacomm_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${ardupilotmegacomm_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${ardupilotmegacomm_VERSION_PATCH}")
set(LIBRARY_OUTPUT_PATH "${ardupilotmegacomm_VERSION_PATCH}")
include (CPack)

# boost
find_package(Boost 1.40 REQUIRED COMPONENTS thread-mt system-mt)
link_directories( ${Boost_LIBRARIES_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

# subdirectories
add_subdirectory(src)
add_subdirectory(test)

# pkgconfig
install(FILES
	ardupilotmegacomm.pc
	DESTINATION lib/pkgconfig
	)
