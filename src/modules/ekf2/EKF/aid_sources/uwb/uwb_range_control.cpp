/****************************************************************************
 * UWB range tightly-coupled fusion for EKF2 (native, uORB-free).
 * Model:    h(x) = ||p - a_i||              (p = vehicle NED pos, a_i = anchor NED pos)
 * Jacobian: H[pos] = (p - a_i)^T / ||p - a_i||   (unit LOS direction)
 * Each tag->anchor range is fused as an independent scalar measurement.
 * See docs/research/control_stack/01_uwb_tightly_coupled_px4.md
 ****************************************************************************/
#include "ekf.h"

#if defined(CONFIG_EKF2_UWB)

matrix::Vector3f Ekf::getUwbAnchorPos(uint8_t anchor_id) const
{
	switch (anchor_id) {
	case 0: return matrix::Vector3f(_params.uwb_anchor_n[0], _params.uwb_anchor_e[0], _params.uwb_anchor_d[0]);
	case 1: return matrix::Vector3f(_params.uwb_anchor_n[1], _params.uwb_anchor_e[1], _params.uwb_anchor_d[1]);
	case 2: return matrix::Vector3f(_params.uwb_anchor_n[2], _params.uwb_anchor_e[2], _params.uwb_anchor_d[2]);
	case 3: return matrix::Vector3f(_params.uwb_anchor_n[3], _params.uwb_anchor_e[3], _params.uwb_anchor_d[3]);
	default: return matrix::Vector3f();
	}
}

bool Ekf::fuseUwbRange(const uwbSample &sample, estimator_aid_source1d_s &aid_src)
{
	// v1 (Phase 1): UWB enhances an existing position estimate; needs a position origin.
	if (!_local_origin_lat_lon.isInitialized()) {
		return false;
	}

	// vehicle NED position relative to the EKF origin at the delayed (fusion) horizon
	float pn;
	float pe;
	_local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pn, pe);

	uint64_t origin_time;
	double ref_lat;
	double ref_lon;
	float ref_alt;
	getEkfGlobalOrigin(origin_time, ref_lat, ref_lon, ref_alt);
	const float pd = ref_alt - (float)_gpos.altitude(); // NED down relative to origin

	const Vector3f pos_ned(pn, pe, pd);
	const Vector3f anchor = getUwbAnchorPos(sample.anchor_id);
	const Vector3f diff = pos_ned - anchor;
	const float predicted = diff.norm();

	if (predicted < 0.1f) {
		return false; // singular Jacobian (vehicle on top of anchor)
	}

	const Vector3f los = diff / predicted;
	VectorState H;
	H.setZero();
	H(State::pos.idx + 0) = los(0);
	H(State::pos.idx + 1) = los(1);
	H(State::pos.idx + 2) = los(2);

	float R = math::max(sq(_params.uwb_noise), 1e-4f);

	if (sample.range_var > 0.f && sample.range_var < 100.f) {
		R = sample.range_var;
	}

	const float innovation = predicted - sample.range;     // h(x) - z (PX4 sign convention)
	const float innovation_var = H.dot(P * H) + R;         // H P H^T + R

	updateAidSourceStatus(aid_src,
			      sample.time_us,
			      sample.range,
			      R,
			      innovation,
			      innovation_var,
			      math::max(_params.uwb_innov_gate, 1.f));

	if (aid_src.innovation_rejected) {
		return false;
	}

	VectorState K = P * H / aid_src.innovation_variance;
	measurementUpdate(K, H, R, aid_src.innovation);

	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;
	_time_last_uwb_fuse = _time_delayed_us;
	_time_last_hor_pos_fuse = _time_delayed_us; // credit UWB as horizontal-position aiding
	return true;
}

void Ekf::controlUwbRangeFusion(const imuSample &imu_delayed)
{
	if (_uwb_buffer == nullptr) {
		return;
	}

	if (_params.uwb_ctrl == 0) {
		if (_control_status.flags.uwb) {
			disableControlStatusUwb();
			ECL_INFO("stopping UWB fusion (disabled)");
		}

		return;
	}

	bool any_fused = false;
	uwbSample sample;

	while (_uwb_buffer->pop_first_older_than(imu_delayed.time_us, &sample)) {
		if (!_control_status.flags.tilt_align) {
			continue;
		}

		if (sample.anchor_id >= (uint8_t)_params.uwb_n_anchors) {
			continue;
		}

		// Needs an initial position. Once UWB is the active source it counts as
		// horizontal aiding (so it self-sustains, e.g. after GPS drops out).
		// Phase 3 will relax this with trilateration-based initialization.
		if (!isHorizontalAidingActive() && !_control_status.flags.uwb) {
			continue;
		}

		any_fused |= fuseUwbRange(sample, _aid_src_uwb[sample.anchor_id]);
	}

	if (any_fused && !_control_status.flags.uwb) {
		ECL_INFO("starting UWB fusion");
		enableControlStatusUwb();
	}

	if (_control_status.flags.uwb && isTimedOut(_time_last_uwb_fuse, _params.reset_timeout_max)) {
		disableControlStatusUwb();
		ECL_INFO("stopping UWB fusion (timeout)");
	}
}

#endif // CONFIG_EKF2_UWB
