#include "ekf.h"
#include "aid_sources/uwb/uwb_range.hpp"

#if defined(CONFIG_EKF2_UWB) && defined(MODULE_NAME)

#include <drivers/drv_hrt.h>

void UwbRange::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	// 1) ingest sensor_uwb into the delay buffer
	sensor_uwb_s msg;

	while (_sensor_uwb_sub.update(&msg)) {
		// anchor index is carried in mac; sim sets mac = anchor_id (0..3)
		if (msg.mac >= (uint16_t)_param_ekf2_uwb_n_anch.get()) {
			continue;
		}

		if (!PX4_ISFINITE(msg.distance) || msg.distance <= 0.f) {
			continue;
		}

		// sensor_uwb has no timestamp_sample -> use timestamp minus a fixed delay
		const int64_t time_us = (int64_t)msg.timestamp - (int64_t)(_param_ekf2_uwb_delay.get() * 1000.f);

		if (time_us < 0) {
			continue;
		}

		UwbSample sample{};
		sample.time_us   = (uint64_t)time_us;
		sample.anchor_id = (uint8_t)msg.mac;
		sample.range     = msg.distance;
		sample.range_var = 0.f; // sensor_uwb carries no variance -> fuse() uses EKF2_UWB_NOISE^2

		_uwb_buffer.push(sample);
		_time_last_buffer_push = imu_delayed.time_us;
	}

	if (_param_ekf2_uwb_ctrl.get() == 0) {
		return;
	}

	// 2) pop everything aligned with the current delayed horizon and fuse per anchor
	UwbSample sample;

	while (_uwb_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {
		estimator_aid_source1d_s &aid_src = _aid_src_uwb[sample.anchor_id];
		fuse(ekf, sample, aid_src);
		aid_src.timestamp = hrt_absolute_time();
		_aid_src_uwb_pub.publish(aid_src);
	}
}

matrix::Vector3f UwbRange::anchorPos(uint8_t anchor_id) const
{
	switch (anchor_id) {
	case 0: return matrix::Vector3f(_param_a0_n.get(), _param_a0_e.get(), _param_a0_d.get());
	case 1: return matrix::Vector3f(_param_a1_n.get(), _param_a1_e.get(), _param_a1_d.get());
	case 2: return matrix::Vector3f(_param_a2_n.get(), _param_a2_e.get(), _param_a2_d.get());
	case 3: return matrix::Vector3f(_param_a3_n.get(), _param_a3_e.get(), _param_a3_d.get());
	default: return matrix::Vector3f();
	}
}

bool UwbRange::fuse(Ekf &ekf, const UwbSample &sample, estimator_aid_source1d_s &aid_src)
{
	// v1: UWB enhances an existing position estimate; it needs a position origin
	// (set by GPS/EV) before it can be fused.
	if (!ekf.global_origin_valid()) {
		return false;
	}

	// vehicle NED position relative to the EKF origin, at the delayed (fusion) horizon
	float pn;
	float pe;
	ekf.global_origin().project(ekf._gpos.latitude_deg(), ekf._gpos.longitude_deg(), pn, pe);

	uint64_t origin_time;
	double ref_lat;
	double ref_lon;
	float ref_alt;
	ekf.getEkfGlobalOrigin(origin_time, ref_lat, ref_lon, ref_alt);
	const float pd = ref_alt - (float)ekf._gpos.altitude(); // NED down relative to origin

	const matrix::Vector3f pos_ned(pn, pe, pd);
	const matrix::Vector3f anchor = anchorPos(sample.anchor_id);
	const matrix::Vector3f diff = pos_ned - anchor;
	const float predicted = diff.norm();

	if (predicted < 0.1f) {
		return false; // singular Jacobian (vehicle on top of anchor)
	}

	// measurement Jacobian: only the position block is non-zero (unit LOS direction)
	const matrix::Vector3f los = diff / predicted;
	Ekf::VectorState H;
	H.setZero();
	H(State::pos.idx + 0) = los(0);
	H(State::pos.idx + 1) = los(1);
	H(State::pos.idx + 2) = los(2);

	// measurement noise: fixed baseline (sensor_uwb carries no variance)
	float R = math::max(sq(_param_ekf2_uwb_noise.get()), 1e-4f);

	if (sample.range_var > 0.f && sample.range_var < 100.f) {
		R = sample.range_var;
	}

	const float innovation = predicted - sample.range;     // h(x) - z  (PX4 sign convention)
	const float innovation_var = H.dot(ekf.P * H) + R;     // H P H^T + R

	ekf.updateAidSourceStatus(aid_src,
				  sample.time_us,
				  sample.range,
				  R,
				  innovation,
				  innovation_var,
				  math::max(_param_ekf2_uwb_gate.get(), 1.f));

	if (aid_src.innovation_rejected) {
		return false;
	}

	Ekf::VectorState K = ekf.P * H / aid_src.innovation_variance;
	ekf.measurementUpdate(K, H, R, aid_src.innovation);

	aid_src.fused = true;
	aid_src.time_last_fuse = ekf._time_delayed_us;
	return true;
}

#endif // CONFIG_EKF2_UWB && MODULE_NAME
