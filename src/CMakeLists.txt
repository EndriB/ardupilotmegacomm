link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( "${PROJECT_SOURCE_DIR}/src" ${Boost_INCLUDE_DIRS} )

# apply patch to APM_BinComm if stamp not found
if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm.patch.stamp)
	exec_program(pwd)
	exec_program(echo ARGS Applying patch to ${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm)
	exec_program(patch ARGS -d ${CMAKE_CURRENT_SOURCE_DIR} -p0 -i ${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm.patch)
	exec_program(touch ARGS ${CMAKE_CURRENT_SOURCE_DIR}/APM_BinComm.patch.stamp)
endif()

add_library(apmcomm SHARED 
	ArdupilotmegaHil.cpp
	AsyncSerial.cpp
	strlcpy.c
	WProgram.cpp
	APM_BinComm/APM_BinComm.cpp
	)
target_link_libraries(apmcomm ${Boost_LIBRARIES} )

set_target_properties(apmcomm PROPERTIES VERSION ${apmcomm_VERISON}
	SOVERSION ${apmcomm_SOVERSION})

install(TARGETS apmcomm DESTINATION lib)
install(FILES
	ArdupilotmegaHil.hpp
	AsyncSerial.hpp
	strlcpy.h
	WProgram.h
	DESTINATION include/apmcomm
	)
install(FILES
	APM_BinComm/APM_BinComm.h
	DESTINATION include/apmcomm/APM_BinComm
	)
install(FILES
	APM_BinComm/protocol/protocol.h
	DESTINATION include/apmcomm/APM_BinComm/protocol
	)
