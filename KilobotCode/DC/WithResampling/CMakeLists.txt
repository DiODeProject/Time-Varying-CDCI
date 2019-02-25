if(ARGOS_BUILD_FOR_SIMULATOR)
include_directories(${CMAKE_SOURCE_DIR}/argos3/plugins/robots/kilobot/control_interface)


add_executable(agentCDCIlocal agentCDCIlocal.c)  
if(UNIX AND NOT APPLE)                              
	target_link_libraries(agentCDCIlocal argos3plugin_simulator_kilolib rt m)
else(UNIX AND NOT APPLE)
	target_link_libraries(agentCDCIlocal argos3plugin_simulator_kilolib)
endif()

endif(ARGOS_BUILD_FOR_SIMULATOR)
