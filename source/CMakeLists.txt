file (GLOB SS_SOURCES
	"*.cpp")
	
file (GLOB SS_HEADERS
	"*.h")
	
message ("BR_BR about to add Box2D subdir")
add_subdirectory (./lib/Box2D)
message ("BR_BR added subdir")

include_directories( ${CURRENT_SOURCE_DIR})

include_directories (./lib/Box2D)
include_directories(./lib/freeglut/include/GL)

add_executable (SimpleSimulation ${SS_SOURCES} ${SS_HEADERS})

message("BR_BR: current source dir: " ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries (SimpleSimulation Box2D)
target_link_libraries (SimpleSimulation ${CMAKE_CURRENT_SOURCE_DIR}/lib/freeglut/lib/freeglut.lib)