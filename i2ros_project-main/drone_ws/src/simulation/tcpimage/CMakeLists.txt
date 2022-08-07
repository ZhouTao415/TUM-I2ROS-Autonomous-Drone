add_library(tcpstreamreader TCPStreamReader.cpp)
target_link_libraries(tcpstreamreader socket++)

add_library(tcpimage TCPImageServer.cpp)
target_link_libraries(tcpimage tcpstreamreader)

add_dependencies(tcpimage libsocket)
add_dependencies(tcpstreamreader libsocket)
