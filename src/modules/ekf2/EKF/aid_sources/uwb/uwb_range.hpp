#ifndef EKF_AID_SOURCES_UWB_RANGE_HPP
#define EKF_AID_SOURCES_UWB_RANGE_HPP

#include "../../common.h"
#include "../../RingBuffer.h"

#if defined(CONFIG_EKF2_UWB) && defined(MODULE_NAME)

# include <px4_platform_common/module_params.h>
# include <matrix/math.hpp>
# include <uORB/PublicationMulti.hpp>
# include <uORB/Subscription.hpp>
# include <uORB/topics/sensor_uwb.h>
# include <uORB/topics/estimator_aid_source1d.h>

class Ekf;

// Tightly-coupled UWB range aiding source.
// Reuses the existing sensor_uwb topic (mac = anchor index, distance = range).
// Each tag->anchor range is fused as a scalar measurement z = ||p - a_i||.
class UwbRange : public ModuleParams
{
public:
	UwbRange() : ModuleParams(nullptr)
	{
		_aid_src_uwb_pub.advertise();
	}

	~UwbRange() = default;

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void updateParameters() { updateParams(); }

private:
	static constexpr uint8_t kMaxAnchors = 4;

	struct UwbSample {
		uint64_t time_us{};    ///< measurement time (uSec), already delay-adjusted
		uint8_t  anchor_id{};
		float    range{};      ///< measured distance (m)
		float    range_var{};  ///< measurement variance (m^2); 0 = use EKF2_UWB_NOISE^2
	};

	matrix::Vector3f anchorPos(uint8_t anchor_id) const;
	bool fuse(Ekf &ekf, const UwbSample &sample, estimator_aid_source1d_s &aid_src);

	RingBuffer<UwbSample> _uwb_buffer{20};
	uint64_t _time_last_buffer_push{0};
	estimator_aid_source1d_s _aid_src_uwb[kMaxAnchors] {};

	uORB::Subscription _sensor_uwb_sub{ORB_ID(sensor_uwb)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _aid_src_uwb_pub{ORB_ID(estimator_aid_src_uwb)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EKF2_UWB_CTRL>)    _param_ekf2_uwb_ctrl,
		(ParamFloat<px4::params::EKF2_UWB_DELAY>) _param_ekf2_uwb_delay,
		(ParamFloat<px4::params::EKF2_UWB_NOISE>) _param_ekf2_uwb_noise,
		(ParamFloat<px4::params::EKF2_UWB_GATE>)  _param_ekf2_uwb_gate,
		(ParamInt<px4::params::EKF2_UWB_N_ANCH>)  _param_ekf2_uwb_n_anch,
		(ParamFloat<px4::params::EKF2_UWB_A0_N>) _param_a0_n,
		(ParamFloat<px4::params::EKF2_UWB_A0_E>) _param_a0_e,
		(ParamFloat<px4::params::EKF2_UWB_A0_D>) _param_a0_d,
		(ParamFloat<px4::params::EKF2_UWB_A1_N>) _param_a1_n,
		(ParamFloat<px4::params::EKF2_UWB_A1_E>) _param_a1_e,
		(ParamFloat<px4::params::EKF2_UWB_A1_D>) _param_a1_d,
		(ParamFloat<px4::params::EKF2_UWB_A2_N>) _param_a2_n,
		(ParamFloat<px4::params::EKF2_UWB_A2_E>) _param_a2_e,
		(ParamFloat<px4::params::EKF2_UWB_A2_D>) _param_a2_d,
		(ParamFloat<px4::params::EKF2_UWB_A3_N>) _param_a3_n,
		(ParamFloat<px4::params::EKF2_UWB_A3_E>) _param_a3_e,
		(ParamFloat<px4::params::EKF2_UWB_A3_D>) _param_a3_d
	)
};

#endif // CONFIG_EKF2_UWB && MODULE_NAME
#endif // EKF_AID_SOURCES_UWB_RANGE_HPP
