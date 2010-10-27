link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( "${PROJECT_SOURCE_DIR}/src" ${Boost_INCLUDE_DIRS} )

# apply patch to APM_BinComm if stamp not found
if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm.patch.stamp)
	exec_program(patch ARGS -d ${CMAKE_CURRENT_SOURCE_DIR} -p0 -i 
		${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm.patch)
	exec_program(touch ARGS ${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm.patch.stamp)
endif()

add_library(ardupilotmegacomm SHARED 
	ArdupilotmegaHil.cpp
	AsyncSerial.cpp
	strlcpy.c
	WProgram.cpp
	APM_BinComm/APM_BinComm.cpp
	)
target_link_libraries(ardupilotmegacomm ${Boost_LIBRARIES} )

set_target_properties(ardupilotmegacomm PROPERTIES VERSION ${ardupilotmegacomm_VERISON}
	SOVERSION ${ardupilotmegacomm_SOVERSION})

install(TARGETS ardupilotmegacomm DESTINATION lib)
install(FILES
	ArdupilotmegaHil.hpp
	AsyncSerial.hpp
	strlcpy.h
	WProgram.h
	DESTINATION include/ardupilotmegacomm
	)
install(FILES
	APM_BinComm/APM_BinComm.h
	DESTINATION include/ardupilotmegacomm/APM_BinComm
	)
install(FILES
	APM_BinComm/protocol/protocol.h
	DESTINATION include/ardupilotmegacomm/APM_BinComm/protocol
	)
