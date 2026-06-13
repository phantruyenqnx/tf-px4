#include "ekf.h"
#include "aid_sources/uwb/uwb_range.hpp"

#if defined(CONFIG_EKF2_UWB) && defined(MODULE_NAME)

void UwbRange::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	(void)ekf;
	(void)imu_delayed;
	// ingest + buffer implemented in Task 4; fusion in Task 5
}

#endif // CONFIG_EKF2_UWB && MODULE_NAME
