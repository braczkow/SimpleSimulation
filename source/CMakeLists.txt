file (GLOB SS_SOURCES
	"*.cpp")
	
file (GLOB SS_HEADERS
	"*.h")
	
message ("BR_BR about to add Box2D subdir")
add_subdirectory (./external/Box2D)
add_subdirectory (./external/jsoncpp)
message ("BR_BR added subdir")

include_directories( ${CURRENT_SOURCE_DIR})

include_directories (./external/Box2D)
include_directories(./external/freeglut/include/GL)
include_directories(./external/jsoncpp)

add_executable (SimpleSimulation ${SS_SOURCES} ${SS_HEADERS})

message("BR_BR: current source dir: " ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries (SimpleSimulation Box2D)
target_link_libraries (SimpleSimulation ${CMAKE_CURRENT_SOURCE_DIR}/external/freeglut/lib/freeglut.lib)
target_link_libraries (SimpleSimulation jsoncpp)



add_custom_command(TARGET SimpleSimulation POST_BUILD        
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  
        "${PROJECT_SOURCE_DIR}/assets/freeglut/bin/freeglut.dll"      
        "${CMAKE_CURRENT_BINARY_DIR}/Debug/freeglut.dll")  


add_custom_command(TARGET SimpleSimulation POST_BUILD        
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  
        "${PROJECT_SOURCE_DIR}/assets/freeglut/bin/freeglut.dll"      
        "${PROJECT_BINARY_DIR}/bin/freeglut.dll")    		


