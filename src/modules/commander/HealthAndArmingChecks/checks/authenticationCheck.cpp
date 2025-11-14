#include "authenticationCheck.hpp"

/* No body definition since we defaulted the class */

void AuthenticationChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (context.isArmed()) {
		return;
	}

	// Fetch the latest authentication status
	uavcan_auth_status_s st{};
	if (_auth_status_sub.copy(&st)) {
		all_nodes_authenticated = st.system_authenticated;
		_auth_failure_mask = static_cast<uint32_t>(st.failed_devices_mask);
	}

	if (!all_nodes_authenticated) {
		/* EVENT
		* @description
		* UAVCAN node authentication has failed.
		*/
		reporter.armingCheckFailure<uint32_t>(
			NavModes::All,
			health_component_t::uavcan_authentication,
			events::ID("check_uavcan_auth_failed"),
			events::Log::Error,
			"UAVCAN node authentication failed (mask {1})",
			_auth_failure_mask);

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: UAVCAN AUTHENTICATION ERROR: mask %lu", _auth_failure_mask);
		}
	}
}
