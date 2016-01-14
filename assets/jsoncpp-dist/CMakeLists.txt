project(jsoncpp)

include_directories( ./json )

set (JSONCPP_SRC
	jsoncpp.cpp)
	
add_library(jsoncpp STATIC
	${JSONCPP_SRC})