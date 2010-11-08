cmake_minimum_required (VERSION 2.6)
project (ardupilotmegacomm)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# The version number.
set(ardupilotmegacomm_VERSION_MAJOR 0)
set(ardupilotmegacomm_VERSION_MINOR 2)
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
include (CPack)

# boost
find_package(Boost 1.40 REQUIRED COMPONENTS thread-mt system-mt)
link_directories( ${Boost_LIBRARIES_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

# the install_name directory component to be used when installing
set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
message(STATUS "install_name ${CMAKE_INSTALL_NAME_DIR}")

# subdirectories
add_subdirectory(src)
add_subdirectory(test)

# pkgconfig
install(FILES
	ardupilotmegacomm.pc
	DESTINATION lib/pkgconfig
	)
