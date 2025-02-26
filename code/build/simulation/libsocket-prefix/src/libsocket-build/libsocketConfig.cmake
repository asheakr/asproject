set(libsocket_INCLUDE_DIRS "/home/akhil/Challenge/asproject/code/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/akhil/Challenge/asproject/code/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
