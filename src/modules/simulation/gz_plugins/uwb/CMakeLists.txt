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

# GTEC UWB ranging plugin (gz-sim System plugin).
# The library is named gtec_uwb_plugin so models load it with
#   <plugin filename="gtec_uwb_plugin" name="gtec::UwbSensorPlugin">
project(gtec_uwb_plugin)

add_library(${PROJECT_NAME} SHARED
    src/UwbSensorPlugin.cc
    src/EcmGeometry.cc
    src/UwbChannelModel.cc
    src/UwbChannelModel_tables.cc
    src/RayObstacleSet.cc
    src/Resolver.cc
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs
    PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::register
    PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
    PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC px4_gz_msgs
)

# ---------------------------------------------------------------------------
# Optional: pure-logic unit tests. OFF by default so the normal `make px4_sitl`
# build is unaffected. Enable with -DUWB_PLUGIN_TESTS=ON (needs GTest).
# ---------------------------------------------------------------------------
option(UWB_PLUGIN_TESTS "Build gtec_uwb_plugin unit tests" OFF)
if(UWB_PLUGIN_TESTS)
    find_package(GTest)
    if(GTest_FOUND)
        add_library(uwb_logic STATIC
            src/UwbChannelModel.cc
            src/UwbChannelModel_tables.cc
            src/RayObstacleSet.cc
            src/Resolver.cc
        )
        target_include_directories(uwb_logic PUBLIC include src)
        target_link_libraries(uwb_logic PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR})

        foreach(t UwbChannelModel RayObstacleSet Resolver)
            add_executable(${t}_TEST test/${t}_TEST.cc)
            target_link_libraries(${t}_TEST GTest::gtest_main uwb_logic)
            add_test(NAME ${t}_TEST COMMAND ${t}_TEST)
        endforeach()

        add_executable(Ranging_TEST test/Ranging_TEST.cc)
        target_link_libraries(Ranging_TEST GTest::gtest_main px4_gz_msgs)
        add_test(NAME Ranging_TEST COMMAND Ranging_TEST)
    endif()
endif()
