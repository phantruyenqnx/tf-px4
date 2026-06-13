############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# Find required packages
find_package(Protobuf)

if (Protobuf_FOUND)
    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})

    # Create library
    add_library(px4_gz_msgs STATIC
        ${PROTO_SRCS}
        ${PROTO_HDRS}
    )

    target_include_directories(px4_gz_msgs
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
            ${Protobuf_INCLUDE_DIRS}
    )
    target_link_libraries(px4_gz_msgs PUBLIC ${PROTOBUF_LIBRARIES})
    if (Protobuf_VERSION VERSION_LESS "3.8")
        target_compile_options(px4_gz_msgs PRIVATE -Wno-error=float-equal)
    endif()

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")

    # Generate a protobuf descriptor set so external tools (e.g. `gz topic -e`)
    # can decode px4.msgs.* custom messages via GZ_DESCRIPTOR_PATH. The C++
    # generation above only emits .pb.{h,cc}; the CLI needs a FileDescriptorSet.
    set(PX4_GZ_MSGS_DESC ${CMAKE_CURRENT_BINARY_DIR}/px4_gz_msgs.desc)
    add_custom_command(
        OUTPUT ${PX4_GZ_MSGS_DESC}
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
                --descriptor_set_out=${PX4_GZ_MSGS_DESC}
                --include_imports
                -I ${CMAKE_CURRENT_SOURCE_DIR}
                ${MSGS_PROTOS}
        DEPENDS ${MSGS_PROTOS}
        COMMENT "Generating px4_gz_msgs protobuf descriptor set (px4_gz_msgs.desc)"
    )
    add_custom_target(px4_gz_msgs_desc ALL DEPENDS ${PX4_GZ_MSGS_DESC})

    # Export the descriptor path for convenience (set GZ_DESCRIPTOR_PATH to this).
    set(PX4_GZ_MSGS_DESC ${PX4_GZ_MSGS_DESC} CACHE INTERNAL "")
endif()
