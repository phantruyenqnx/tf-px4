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

#include "GZHILBridge.hpp"

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>

#include <cmath>
#include <iostream>

using namespace custom;

// Register plugin with Gz
GZ_ADD_PLUGIN(
	GZHILBridge,
	gz::sim::System,
	GZHILBridge::ISystemConfigure,
	GZHILBridge::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GZHILBridge, "custom::GZHILBridge")

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

GZHILBridge::~GZHILBridge()
{
	_running = false;

	if (_reader_thread.joinable()) {
		_reader_thread.join();
	}

	closeTransport();
}

// ---------------------------------------------------------------------------
// ISystemConfigure
// ---------------------------------------------------------------------------

void GZHILBridge::Configure(
	const gz::sim::Entity &entity,
	const std::shared_ptr<const sdf::Element> &_sdf,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &_eventMgr)
{
	// Gz Harmonic with merge='true' can call Configure() twice — guard against double-init
	if (_configured) {
		std::cerr << "[GZHILBridge] Configure() called again — ignoring (merge='true' duplicate)\n";
		return;
	}

	_configured = true;
	_entity = entity;

	// Read SDF parameters
	if (_sdf->HasElement("serial_device")) {
		_serial_device = _sdf->Get<std::string>("serial_device");
	}

	if (_sdf->HasElement("baud_rate")) {
		_baud_rate = _sdf->Get<int>("baud_rate");
	}

	if (_sdf->HasElement("model_name")) {
		_model_name = _sdf->Get<std::string>("model_name");
	}

	if (_sdf->HasElement("world_name")) {
		_world_name = _sdf->Get<std::string>("world_name");
	}

	if (_sdf->HasElement("tcp_host")) {
		_tcp_host = _sdf->Get<std::string>("tcp_host");
	}

	if (_sdf->HasElement("tcp_port")) {
		_tcp_port = _sdf->Get<int>("tcp_port");
	}

	if (_tcp_host.empty()) {
		std::cout << "[GZHILBridge] transport=serial device=" << _serial_device
			  << " baud=" << _baud_rate << " model=" << _model_name << "\n";
	} else {
		std::cout << "[GZHILBridge] transport=tcp host=" << _tcp_host
			  << " port=" << _tcp_port << " model=" << _model_name << "\n";
	}

	// Advertise actuator topic to drive Gz motors
	const std::string actuator_topic = "/" + _model_name + "/command/motor_speed";
	_actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);
	std::cout << "[GZHILBridge] advertising actuators on " << actuator_topic << "\n";

	// Subscribe to sensor topics (same paths as GZBridge)
	const std::string base = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/";

	_node.Subscribe(base + "imu_sensor/imu",
		&GZHILBridge::imuCallback, this);

	_node.Subscribe(base + "magnetometer_sensor/magnetometer",
		&GZHILBridge::magnetometerCallback, this);

	_node.Subscribe(base + "air_pressure_sensor/air_pressure",
		&GZHILBridge::barometerCallback, this);

	_node.Subscribe(base + "navsat_sensor/navsat",
		&GZHILBridge::navSatCallback, this);

	std::cout << "[GZHILBridge] subscribed to sensors under " << base << "\n";

	// Open transport (serial or UDP)
	if (!openTransport()) {
		std::cerr << "[GZHILBridge] WARNING: could not open transport"
			  << " — will retry each PostUpdate\n";
	}

	// Start reader thread
	_running = true;
	_reader_thread = std::thread(&GZHILBridge::readerThread, this);
}

// ---------------------------------------------------------------------------
// ISystemPostUpdate
// ---------------------------------------------------------------------------

void GZHILBridge::PostUpdate(
	const gz::sim::UpdateInfo &_info,
	const gz::sim::EntityComponentManager &_ecm)
{
	(void)_ecm;

	if (_info.paused) { return; }

	// Retry transport open if not connected — throttled to once every 2s
	const double sim_s = std::chrono::duration<double>(_info.simTime).count();

	{
		std::lock_guard<std::mutex> lock(_fd_mutex);

		if (_serial_fd < 0 && _tcp_fd < 0) {
			if (sim_s - _last_reconnect_s >= 2.0) {
				_last_reconnect_s = sim_s;
				if (openTransport()) {
					std::cout << "[GZHILBridge] reconnected\n";
				}
			}
			return;
		}
	}

	// Send heartbeat at ~1 Hz so PX4 activates its HIL MAVLink streams
	if (sim_s - _last_heartbeat_s >= 1.0) {
		sendHeartbeat();
		_last_heartbeat_s = sim_s;
	}
}

// ---------------------------------------------------------------------------
// Transport open / close / write (serial or TCP)
// ---------------------------------------------------------------------------

bool GZHILBridge::openTransport()
{
	return _tcp_host.empty() ? openSerial() : openTcp();
}

void GZHILBridge::closeTransport()
{
	closeSerial();
	closeTcp();
}

bool GZHILBridge::writeMavlink(const mavlink_message_t &msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	std::lock_guard<std::mutex> lock(_fd_mutex);

	if (_serial_fd >= 0) {
		if (write(_serial_fd, buf, len) != static_cast<ssize_t>(len)) {
			std::cerr << "[GZHILBridge] serial write failed — closing\n";
			closeSerial();
			return false;
		}
		return true;
	}

	if (_tcp_fd >= 0) {
		const ssize_t sent = send(_tcp_fd, buf, len, MSG_NOSIGNAL);
		if (sent != static_cast<ssize_t>(len)) {
			closeTcp();  // PostUpdate will reconnect at next 2s interval
			return false;
		}
		return true;
	}

	return false;
}

// Serial

bool GZHILBridge::openSerial()
{
	_serial_fd = open(_serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		return false;
	}

	struct termios tty {};
	tcgetattr(_serial_fd, &tty);

	speed_t speed = B921600;

	if (_baud_rate == 57600)       { speed = B57600;  }
	else if (_baud_rate == 115200) { speed = B115200; }
	else if (_baud_rate == 921600) { speed = B921600; }

	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
	tty.c_iflag = IGNBRK;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 0;
	tcsetattr(_serial_fd, TCSANOW, &tty);

	std::cout << "[GZHILBridge] Opened serial " << _serial_device << "\n";
	return true;
}

void GZHILBridge::closeSerial()
{
	if (_serial_fd >= 0) {
		close(_serial_fd);
		_serial_fd = -1;
	}
}

// TCP (connects to mavlink-routerd TCP server)

bool GZHILBridge::openTcp()
{
	_tcp_fd = socket(AF_INET, SOCK_STREAM, 0);

	if (_tcp_fd < 0) {
		std::cerr << "[GZHILBridge] TCP socket() failed\n";
		return false;
	}

	struct sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port   = htons(static_cast<uint16_t>(_tcp_port));
	inet_pton(AF_INET, _tcp_host.c_str(), &addr.sin_addr);

	if (connect(_tcp_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
		std::cerr << "[GZHILBridge] TCP connect to " << _tcp_host << ":" << _tcp_port << " failed\n";
		close(_tcp_fd);
		_tcp_fd = -1;
		return false;
	}

	// Keep socket blocking — send() at high rate needs reliable delivery
	// (non-blocking would silently drop packets on EAGAIN)
	std::cout << "[GZHILBridge] TCP connected to " << _tcp_host << ":" << _tcp_port << "\n";
	return true;
}

void GZHILBridge::closeTcp()
{
	if (_tcp_fd >= 0) {
		close(_tcp_fd);
		_tcp_fd = -1;
	}
}

// ---------------------------------------------------------------------------
// Heartbeat — must be sent at ≥1 Hz so PX4 activates HIL MAVLink streams
// ---------------------------------------------------------------------------

void GZHILBridge::sendHeartbeat()
{
	mavlink_heartbeat_t hb{};
	hb.type           = MAV_TYPE_GCS;
	hb.autopilot      = MAV_AUTOPILOT_INVALID;
	hb.base_mode      = 0;
	hb.custom_mode    = 0;
	hb.system_status  = MAV_STATE_ACTIVE;

	mavlink_message_t msg;
	mavlink_msg_heartbeat_encode(255, MAV_COMP_ID_MISSIONPLANNER, &msg, &hb);
	writeMavlink(msg);
}

// ---------------------------------------------------------------------------
// Sensor callbacks — encode MAVLink and send to board
// ---------------------------------------------------------------------------

// FLU → FRD: negate Y and Z axes (Gz body frame → PX4 body frame)
static inline gz::math::Vector3d flu_to_frd(const gz::math::Vector3d &v)
{
	return { v.X(), -v.Y(), -v.Z() };
}

void GZHILBridge::imuCallback(const gz::msgs::IMU &msg)
{
	{ std::lock_guard<std::mutex> lock(_fd_mutex); if (_serial_fd < 0 && _tcp_fd < 0) { return; } }

	const uint64_t time_us = static_cast<uint64_t>(
		msg.header().stamp().sec()) * 1000000ULL +
		static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;

	const gz::math::Vector3d accel = flu_to_frd({
		msg.linear_acceleration().x(),
		msg.linear_acceleration().y(),
		msg.linear_acceleration().z()
	});

	const gz::math::Vector3d gyro = flu_to_frd({
		msg.angular_velocity().x(),
		msg.angular_velocity().y(),
		msg.angular_velocity().z()
	});

	mavlink_hil_sensor_t sensor{};
	sensor.time_usec      = time_us;
	sensor.xacc           = static_cast<float>(accel.X());
	sensor.yacc           = static_cast<float>(accel.Y());
	sensor.zacc           = static_cast<float>(accel.Z());
	sensor.xgyro          = static_cast<float>(gyro.X());
	sensor.ygyro          = static_cast<float>(gyro.Y());
	sensor.zgyro          = static_cast<float>(gyro.Z());
	// mag fields left zero here — magnetometerCallback fills them in a separate message
	sensor.abs_pressure   = 0.0f;
	sensor.diff_pressure  = 0.0f;
	sensor.pressure_alt   = 0.0f;
	sensor.temperature    = _temperature;
	// bits 0-2: accel valid, bits 3-5: gyro valid
	sensor.fields_updated = 0b000111111u;
	sensor.id             = 0;

	mavlink_message_t msg_out{};
	mavlink_msg_hil_sensor_encode(1, 200, &msg_out, &sensor);
	writeMavlink(msg_out);
}

void GZHILBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{
	{ std::lock_guard<std::mutex> lock(_fd_mutex); if (_serial_fd < 0 && _tcp_fd < 0) { return; } }

	const uint64_t time_us = static_cast<uint64_t>(
		msg.header().stamp().sec()) * 1000000ULL +
		static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;

	// Gz magnetometer plugin publishes in a left-handed frame (same quirk as GZBridge)
	// x_frd = -y_gz, y_frd = -x_gz, z_frd = z_gz
	const float mx = static_cast<float>(-msg.field_tesla().y());
	const float my = static_cast<float>(-msg.field_tesla().x());
	const float mz = static_cast<float>( msg.field_tesla().z());

	mavlink_hil_sensor_t sensor{};
	sensor.time_usec      = time_us;
	sensor.xmag           = mx;
	sensor.ymag           = my;
	sensor.zmag           = mz;
	sensor.temperature    = _temperature;
	// bits 6-8: mag valid
	sensor.fields_updated = 0b111000000u;
	sensor.id             = 0;

	mavlink_message_t msg_out{};
	mavlink_msg_hil_sensor_encode(1, 200, &msg_out, &sensor);
	writeMavlink(msg_out);
}

void GZHILBridge::barometerCallback(const gz::msgs::FluidPressure &msg)
{
	{ std::lock_guard<std::mutex> lock(_fd_mutex); if (_serial_fd < 0 && _tcp_fd < 0) { return; } }

	const uint64_t time_us = static_cast<uint64_t>(
		msg.header().stamp().sec()) * 1000000ULL +
		static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;

	const float pressure_pa = static_cast<float>(msg.pressure());

	// ISA pressure altitude: p0 * (1 - alt/44330)^5.255  → invert for alt
	// Simple approximation used by SimulatorMavlink
	constexpr float p0 = 101325.0f;
	const float pressure_alt = 44330.0f * (1.0f - powf(pressure_pa / p0, 1.0f / 5.255f));

	mavlink_hil_sensor_t sensor{};
	sensor.time_usec      = time_us;
	sensor.abs_pressure   = pressure_pa / 100.0f;  // Pa → hPa (mbar)
	sensor.diff_pressure  = 0.0f;
	sensor.pressure_alt   = pressure_alt;
	sensor.temperature    = _temperature;
	// SensorSource::BARO = 0b1101000000000 (bits 9=abs_pressure, 11=pressure_alt, 12=temperature)
	sensor.fields_updated = 0b1101000000000u;
	sensor.id             = 0;

	mavlink_message_t msg_out{};
	mavlink_msg_hil_sensor_encode(1, 200, &msg_out, &sensor);
	writeMavlink(msg_out);
}

void GZHILBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
	{ std::lock_guard<std::mutex> lock(_fd_mutex); if (_serial_fd < 0 && _tcp_fd < 0) { return; } }

	const uint64_t time_us = static_cast<uint64_t>(
		msg.header().stamp().sec()) * 1000000ULL +
		static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;

	// ENU → NED velocity: vn=north_enu, ve=east_enu, vd=-up_enu
	const float vel_n = static_cast<float>(msg.velocity_north());
	const float vel_e = static_cast<float>(msg.velocity_east());
	const float vel_d = static_cast<float>(-msg.velocity_up());

	const float speed  = sqrtf(vel_n * vel_n + vel_e * vel_e);
	const float cog_deg = atan2f(vel_e, vel_n) * 180.0f / static_cast<float>(M_PI);
	const float cog_cdeg = (cog_deg < 0.0f ? cog_deg + 360.0f : cog_deg) * 100.0f;

	mavlink_hil_gps_t gps{};
	gps.time_usec          = time_us;
	gps.fix_type           = 3;   // 3D fix
	gps.lat                = static_cast<int32_t>(msg.latitude_deg()  * 1e7);
	gps.lon                = static_cast<int32_t>(msg.longitude_deg() * 1e7);
	gps.alt                = static_cast<int32_t>(msg.altitude() * 1e3);  // m → mm
	gps.eph                = 100;   // cm
	gps.epv                = 100;   // cm
	gps.vel                = static_cast<uint16_t>(speed * 100.0f);        // m/s → cm/s
	gps.vn                 = static_cast<int16_t>(vel_n * 100.0f);
	gps.ve                 = static_cast<int16_t>(vel_e * 100.0f);
	gps.vd                 = static_cast<int16_t>(vel_d * 100.0f);
	gps.cog                = static_cast<uint16_t>(cog_cdeg);
	gps.satellites_visible = 12;
	gps.id                 = 0;

	mavlink_message_t msg_out{};
	mavlink_msg_hil_gps_encode(1, 200, &msg_out, &gps);
	writeMavlink(msg_out);
}

// ---------------------------------------------------------------------------
// Reader thread (Board → Gz)
// ---------------------------------------------------------------------------

void GZHILBridge::readerThread()
{
	mavlink_message_t msg;
	mavlink_status_t  status;
	uint8_t           buf[512];

	while (_running) {
		// Snapshot fd under lock — do NOT hold lock during blocking read()
		int fd;
		{
			std::lock_guard<std::mutex> lock(_fd_mutex);
			fd = (_serial_fd >= 0) ? _serial_fd : _tcp_fd;
		}

		if (fd < 0) {
			// Wait for PostUpdate to reconnect
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			continue;
		}

		const ssize_t n = read(fd, buf, sizeof(buf));

		if (n == 0) {
			std::cerr << "[GZHILBridge] connection closed — will reconnect\n";
			{ std::lock_guard<std::mutex> lock(_fd_mutex); closeTransport(); }
			std::this_thread::sleep_for(std::chrono::seconds(2));
			continue;
		}

		if (n < 0) {
			if (errno == EINTR) { continue; }
			std::cerr << "[GZHILBridge] read error " << errno << " — will reconnect\n";
			{ std::lock_guard<std::mutex> lock(_fd_mutex); closeTransport(); }
			std::this_thread::sleep_for(std::chrono::seconds(2));
			continue;
		}

		for (ssize_t i = 0; i < n; i++) {
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
				if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
					mavlink_hil_actuator_controls_t ctrl;
					mavlink_msg_hil_actuator_controls_decode(&msg, &ctrl);
					handleActuatorControls(ctrl);
				}
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Handle HIL_ACTUATOR_CONTROLS → publish Gz motor speeds
// ---------------------------------------------------------------------------

void GZHILBridge::handleActuatorControls(const mavlink_hil_actuator_controls_t &ctrl)
{
	{
		std::lock_guard<std::mutex> lock(_actuator_mutex);
		std::copy(std::begin(ctrl.controls), std::end(ctrl.controls),
			  std::begin(_actuator_controls));
		_armed = (ctrl.mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
		_actuator_received = true;
	}

	// HIL_ACTUATOR_CONTROLS values are normalized [0, 1].
	// MulticopterMotorModel (motorType=velocity, maxRotVelocity=1000) expects rad/s.
	// Scale: rad/s = control * maxRotVelocity
	constexpr float MAX_ROT_VELOCITY = 1000.0f;

	gz::msgs::Actuators actuators_msg;
	actuators_msg.mutable_velocity()->Resize(4, 0.0f);

	{
		std::lock_guard<std::mutex> lock(_actuator_mutex);

		for (int i = 0; i < 4; i++) {
			const float rad_s = _armed ? _actuator_controls[i] * MAX_ROT_VELOCITY : 0.0f;
			actuators_msg.set_velocity(i, rad_s);
		}
	}

	_actuators_pub.Publish(actuators_msg);
}
