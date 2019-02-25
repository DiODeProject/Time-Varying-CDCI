if(ARGOS_BUILD_FOR_SIMULATOR)
include_directories(${CMAKE_SOURCE_DIR}/argos3/plugins/robots/kilobot/control_interface)

foreach(r 1) 
add_executable(agentCDCIlocal_r${r} agentCDCIlocal.h agentCDCIlocal_r${r}.c)
if(UNIX AND NOT APPLE)                              
target_link_libraries(agentCDCIlocal_r${r}  argos3plugin_simulator_kilolib rt m)
else(UNIX AND NOT APPLE)
target_link_libraries(agentCDCIlocal_r{r}  argos3plugin_simulator_kilolib)
endif() 
endforeach()


endif(ARGOS_BUILD_FOR_SIMULATOR)
