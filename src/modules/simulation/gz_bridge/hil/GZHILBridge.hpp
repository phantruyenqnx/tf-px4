/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file GZHILBridge.hpp
 *
 * Gazebo Harmonic HIL (Hardware-in-the-Loop) bridge plugin.
 *
 * Supports two transport modes (selected by SDF params):
 *
 *   Serial mode (default): GZHILBridge owns /dev/ttyACM0 exclusively.
 *     <serial_device>/dev/ttyACM0</serial_device>
 *     <baud_rate>921600</baud_rate>
 *
 *   TCP mode: mavlink-routerd owns the serial port; GZHILBridge connects
 *     to routerd's built-in TCP server (port 5760). QGC connects via UDP.
 *     Launch: mavlink-routerd -e 172.17.128.1:14550 /dev/ttyACM0:921600
 *     <tcp_host>127.0.0.1</tcp_host>
 *     <tcp_port>5760</tcp_port>
 *
 * Board must have SYS_HITL=1 set before connecting.
 */

#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

// mavlink C library (header-only)
#include <development/mavlink.h>

namespace custom
{

class GZHILBridge :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPostUpdate
{
public:
	GZHILBridge() = default;
	~GZHILBridge() override;

	// ISystemConfigure: called once at world load
	void Configure(
		const gz::sim::Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		gz::sim::EntityComponentManager &_ecm,
		gz::sim::EventManager &_eventMgr) override;

	// ISystemPostUpdate: called after every physics step
	void PostUpdate(
		const gz::sim::UpdateInfo &_info,
		const gz::sim::EntityComponentManager &_ecm) override;

private:
	// --- Transport (serial or UDP) ---
	bool openTransport();
	void closeTransport();
	bool writeMavlink(const mavlink_message_t &msg);

	// Serial
	bool openSerial();
	void closeSerial();

	// TCP (connect to mavlink-routerd TCP server on port 5760)
	bool openTcp();
	void closeTcp();

	// --- Sensor topic callbacks (encode MAVLink and write serial) ---
	void imuCallback(const gz::msgs::IMU &msg);
	void magnetometerCallback(const gz::msgs::Magnetometer &msg);
	void barometerCallback(const gz::msgs::FluidPressure &msg);
	void navSatCallback(const gz::msgs::NavSat &msg);

	// --- Reader thread (Board → Gz) ---
	void readerThread();
	void handleActuatorControls(const mavlink_hil_actuator_controls_t &msg);

	// --- Config (from SDF) ---
	std::string _serial_device{"/dev/ttyACM0"};
	int         _baud_rate{921600};
	std::string _model_name{"x500_hitl"};
	std::string _world_name{"default"};

	// TCP config (optional — if set, TCP is used instead of serial)
	std::string _tcp_host{};     // empty = serial mode; set to connect to mavlink-routerd
	int         _tcp_port{5760}; // routerd TCP server port (default 5760)

	// --- Serial fd ---
	int _serial_fd{-1};

	// --- TCP socket ---
	int _tcp_fd{-1};

	// --- Gz transport ---
	gz::transport::Node _node;
	gz::transport::Node::Publisher _actuators_pub;

	// --- Reader thread ---
	std::thread      _reader_thread;
	std::atomic<bool> _running{false};

	// --- fd mutex (protects _serial_fd and _tcp_fd across threads) ---
	std::mutex _fd_mutex;

	// --- Actuator state (shared between reader thread and PostUpdate) ---
	std::mutex _actuator_mutex;
	float      _actuator_controls[16]{};
	bool       _armed{false};
	bool       _actuator_received{false};

	// --- Sensor state (updated by callbacks, read by PostUpdate) ---
	float _temperature{288.15f};  // cached from baro, reused by IMU fields

	// --- Heartbeat ---
	void sendHeartbeat();
	double _last_heartbeat_s{0.0};

	// --- Reconnect throttle ---
	double _last_reconnect_s{0.0};

	// --- Configure guard (merge='true' can trigger Configure twice) ---
	bool _configured{false};

	// --- Gz entity ---
	gz::sim::Entity _entity{gz::sim::kNullEntity};
};

} // namespace custom
