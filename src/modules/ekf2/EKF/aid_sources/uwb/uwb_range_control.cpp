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
	// vehicle NED position in the EKF local frame at the delayed (fusion) horizon.
	// getLocalHorizontalPosition() works with or without a global (GPS) origin, so this
	// path also supports GPS-denied / UWB-only operation (Phase 3).
	// vehicle NED position in the EKF local frame at the delayed (fusion) horizon.
	// getLocalHorizontalPosition() works with or without a global (GPS) origin, so this
	// path also supports GPS-denied / UWB-only operation.
	const Vector2f pos_ne = getLocalHorizontalPosition();
	const float pd = -(float)(_gpos.altitude() - getEkfGlobalOriginAltitude()); // NED down rel. origin
	const Vector3f pos_ned(pos_ne(0), pos_ne(1), pd);
	const Vector3f anchor = getUwbAnchorPos(sample.anchor_id);
	const Vector3f diff = pos_ned - anchor;
	const float predicted = diff.norm();

	if (predicted < 0.1f) {
		return false; // singular Jacobian (vehicle on top of anchor)
	}

	// Full 3-D range fusion. The anchors are NON-coplanar (staggered heights), so the range gives
	// vertical observability too: UWB constrains N/E/D. Height stays well-conditioned because the
	// LOS Jacobian has a real vertical component.
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

bool Ekf::tryInitUwb()
{
	// 2-D horizontal trilateration: solve for (N, E) only (height stays on baro).
	// This handles coplanar anchors (e.g. all at the same height) which make a 3-D
	// solve singular, and matches the fact that we only resetHorizontalPositionTo().
	// Needs >= 3 anchors.
	const int n = math::min((int)_params.uwb_n_anchors, 4);

	if (n < 3) {
		return false;
	}

	// require a recent range from every configured anchor
	for (int i = 0; i < n; i++) {
		if (!isRecent(_uwb_latest[i].time_us, (uint64_t)5e5) || (_uwb_latest[i].range <= 0.f)) {
			return false;
		}
	}

	// remove the vertical leg using the current height estimate (down, NED rel. origin)
	const float pd = -(float)(_gpos.altitude() - getEkfGlobalOriginAltitude());

	const Vector3f a0 = getUwbAnchorPos(0);
	const float rh0_sq = sq(_uwb_latest[0].range) - sq(pd - a0(2)); // horizontal range^2 to anchor 0

	// Linearize, subtract anchor 0:  2 (c_i - c_0)^T q = (|c_i|^2 - |c_0|^2) - (rh_i^2 - rh_0^2)
	// Solve the (n-1 x 2) least-squares system via the 2x2 normal equations A^T A q = A^T b.
	matrix::SquareMatrix<float, 2> AtA;
	Vector2f Atb;
	AtA.setZero();
	Atb.setZero();

	for (int i = 1; i < n; i++) {
		const Vector3f ai = getUwbAnchorPos(i);
		const float rhi_sq = sq(_uwb_latest[i].range) - sq(pd - ai(2));
		const Vector2f arow(2.f * (ai(0) - a0(0)), 2.f * (ai(1) - a0(1)));
		const float brow = (ai(0) * ai(0) + ai(1) * ai(1) - a0(0) * a0(0) - a0(1) * a0(1))
				   - (rhi_sq - rh0_sq);

		AtA(0, 0) += arow(0) * arow(0);
		AtA(0, 1) += arow(0) * arow(1);
		AtA(1, 0) += arow(1) * arow(0);
		AtA(1, 1) += arow(1) * arow(1);
		Atb(0) += arow(0) * brow;
		Atb(1) += arow(1) * brow;
	}

	matrix::SquareMatrix<float, 2> AtA_inv;

	if (!matrix::inv(AtA, AtA_inv)) {
		return false; // anchors collinear / degenerate horizontal geometry
	}

	const Vector2f q = AtA_inv * Atb; // vehicle NE position in the anchor (local) frame

	if (!q.isAllFinite()) {
		return false;
	}

	const float var = sq(math::max(_params.uwb_noise, 0.1f)) * 4.f;
	resetHorizontalPositionTo(q, Vector2f(var, var));

	enableControlStatusUwb();
	_time_last_uwb_fuse = _time_delayed_us;
	_time_last_hor_pos_fuse = _time_delayed_us;
	ECL_INFO("UWB position initialized by trilateration");
	return true;
}

uint8_t Ekf::countRecentUwbAnchors() const
{
	uint8_t n = 0;
	const int n_anch = math::min((int)_params.uwb_n_anchors, 4);

	for (int i = 0; i < n_anch; i++) {
		if (_uwb_latest[i].range > 0.f && isRecent(_uwb_latest[i].time_us, (uint64_t)5e5)) {
			n++;
		}
	}

	return n;
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

		_uwb_latest[sample.anchor_id] = sample; // keep newest per anchor for cold-start init

		// Needs an initial position. With GPS/EV active, refine it. With no other
		// horizontal aiding (indoor GPS-denied), bootstrap it via trilateration.
		// Once UWB is the active source it counts as horizontal aiding, so it
		// self-sustains (e.g. after GPS drops out).
		if (!isHorizontalAidingActive() && !_control_status.flags.uwb) {
			if (!tryInitUwb()) {
				continue;
			}
		}

		if (fuseUwbRange(sample, _aid_src_uwb[sample.anchor_id])) {
			any_fused = true;
			_uwb_reject_count = 0;

		} else if (_aid_src_uwb[sample.anchor_id].innovation_rejected && _uwb_reject_count < 250) {
			_uwb_reject_count++;
		}
	}

	// Re-acquisition: after flying out of UWB range the EKF drifts on GPS; when the ranges return
	// the UWB innovations are large and get gated out (chicken-and-egg, ~73% rejected in tests). If
	// UWB is meant to dominate (EKF2_UWB_GPS) and >=3 anchors are ranging but fusion keeps being
	// rejected, snap the horizontal position to the UWB trilateration solution so UWB re-locks; the
	// R-inflation on GNSS then keeps the estimate on the UWB-defined frame for the landing.
	if (_params.uwb_gps != 0 && _uwb_reject_count >= 5 && countRecentUwbAnchors() >= 3) {
		if (tryInitUwb()) {
			_uwb_reject_count = 0;
			ECL_INFO("UWB re-acquired by trilateration reset (EKF had drifted)");
		}
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
