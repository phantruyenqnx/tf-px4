############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

project(GZHILBridge)

# GZHILBridge is a Gz Harmonic System plugin (.so).
# It is self-contained: depends only on gz-sim, gz-transport, gz-plugin,
# and the mavlink C header library (header-only, vendored via PX4 build).
# No PX4 uORB or NuttX dependency.

if(NOT gz-transport_FOUND OR NOT gz-sim_FOUND OR NOT gz-plugin_FOUND)
    message(STATUS "GZHILBridge: skipping — gz-transport/gz-sim/gz-plugin not found")
    return()
endif()

add_library(GZHILBridge SHARED
    GZHILBridge.cpp
)
 
# Ensure mavlink C headers (development dialect) are generated first
add_dependencies(GZHILBridge mavlink_c_generate)
 
target_include_directories(GZHILBridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    # mavlink C library (header-only), provided by PX4 build system
    ${CMAKE_BINARY_DIR}/mavlink
    ${CMAKE_BINARY_DIR}/mavlink/development
)

target_link_libraries(GZHILBridge
    gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
    gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
    gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
)

# MAVLink headers trigger -Waddress-of-packed-member; suppress for this target only
target_compile_options(GZHILBridge PRIVATE -Wno-address-of-packed-member)

# Output alongside other Gz plugins so GZ_SIM_SYSTEM_PLUGIN_PATH needs only one entry
if(DEFINED PX4_GZ_PLUGIN_OUTPUT_DIR)
    set_target_properties(GZHILBridge PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${PX4_GZ_PLUGIN_OUTPUT_DIR}
    )
endif()

