#include "ekf.h"
#include "aid_sources/uwb/uwb_range.hpp"

#if defined(CONFIG_EKF2_UWB) && defined(MODULE_NAME)

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
		_aid_src_uwb_pub.publish(aid_src);
	}
}

bool UwbRange::fuse(Ekf &ekf, const UwbSample &sample, estimator_aid_source1d_s &aid_src)
{
	(void)ekf;

	// Task 4 stub: surface the popped range on the aid-source topic so the
	// ingest -> buffer -> pop path is observable. Real fusion is Task 5.
	aid_src = {};
	aid_src.timestamp_sample = sample.time_us;
	aid_src.observation = sample.range;

	return false;
}

#endif // CONFIG_EKF2_UWB && MODULE_NAME
