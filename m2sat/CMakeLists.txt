cmake_minimum_required(VERSION 3.10)

# Set project name
project(m2sat)
set(CMAKE_BUILD_TYPE RELEASE)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(sources 
    src/imu.cpp 
    src/tic.cpp   
    src/main.cpp)

# Add module subdirectories
add_subdirectory(controllers)
add_subdirectory(vnproglib/cpp libvncxx)


find_package(ZeroMQ REQUIRED) 
find_package(Eigen3 REQUIRED) # Find Eigen3
find_package(Protobuf REQUIRED) # Find Protobuf

# Protobuffer class generation
set(PROTO_FILES proto/telemetry.proto) # Define the .proto file
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES}) # Generate C++ files from .proto

# Generate Python Protobuf files
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/telemetry_pb2.py
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --proto_path=${CMAKE_SOURCE_DIR} --python_out=${CMAKE_BINARY_DIR} ${PROTO_FILES}
    DEPENDS ${PROTO_FILES}
)
add_custom_target(proto_py ALL DEPENDS ${CMAKE_BINARY_DIR}/telemetry_pb2.py)


include_directories(include ${EIGEN3_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/vnproglib/cpp/include
    cppzmq)

    # Add the executable
add_executable(m2sat ${sources} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(m2sat PRIVATE controllers Eigen3::Eigen protobuf::libprotobuf libvncxx zmq)
