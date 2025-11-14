#pragma once

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/uavcan_auth_status.h>

class AuthenticationChecks : public HealthAndArmingCheckBase
{
public:
	AuthenticationChecks() = default;
	~AuthenticationChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;
private:
	uORB::Subscription _auth_status_sub{ORB_ID(uavcan_auth_status)};

	uint32_t _auth_failure_mask;
	bool all_nodes_authenticated = true;

	// void updateParams() override;

	/* We may not need any parameters here.*/
	// DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase, (ParamInt<px4::params::UAVCAN_AUTH_MODE>) _param_uavcan_auth_mode );
};
