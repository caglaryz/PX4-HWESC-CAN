/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file hwesc.cpp
 *
 * @author Caglar Yilmaz <yilmaz.caglar@tubitak.gov.tr>
 */

#include "hwesc.hpp"
#include <errno.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <lib/atmosphere/atmosphere.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanHwescController::UavcanHwescController(uavcan::INode &node) :
	ModuleParams(nullptr),
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_sub_status_msg1(node),
	_sub_status_msg2(node),
	_pub_get_esc_id(node),
	_sub_get_esc_id(node),
	_timer(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin);
	_pub_get_esc_id.setPriority(uavcan::TransferPriority::NumericallyMin);
}

/**
 * Initialize the UAVCAN ESC controller node
 *
 * Fetch Actuator UAVCAN_ESC_IFACE parameter for iface mask
 *
 * Fetch Actuator UAVCAN_EC_FUNCx parameters -> _param_handles[]
 *
 * Initialize GetEscID client
 *
 * Subscribe to StatusMsg1, and StatusMsg2 messages
 *
 * @return	OK on success, negative errno on failure
 */
int
UavcanHwescController::init()
{
	int32_t iface_mask{0xFF};

	if (param_get(param_find("UAVCAN_ESC_IFACE"), &iface_mask) == OK) {
		_uavcan_pub_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
	}

	char param_name[17];

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		snprintf(param_name, sizeof(param_name), "UAVCAN_EC_FUNC%d", i + 1);
		_param_handles[i] = param_find(param_name);

		int32_t val = 0;

		if (param_get(_param_handles[i], &val) == 0) {
			if (val > 0) {
				_uavcan_rotor_count = i + 1;
			}
		}
	}

	int res = _sub_get_esc_id.start(GetEscIDCbBinder(this, &UavcanHwescController::get_esc_id_cb));
       	if (res < 0) return res;

       	res = _sub_status_msg1.start(StatusMsg1CbBinder(this, &UavcanHwescController::status_msg1_sub_cb));
       	if (res < 0) return res;

       	res = _sub_status_msg2.start(StatusMsg2CbBinder(this, &UavcanHwescController::status_msg2_sub_cb));
       	if (res < 0) return res;

       	_timer.setCallback(TimerCbBinder(this, &UavcanHwescController::periodic_update));
       	_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_MSG_RATE_HZ));

       	_esc_status_pub.advertise();

	// Initialize banned node ID list (temporary)
	memset(banned_node_ids, 0, sizeof(banned_node_ids));
	banned_node_ids[1]  = true;
	banned_node_ids[10] = true;
	PX4_INFO("HWESC banned node IDs: 1, 10");

	return OK;
}

/**
 * Check if the given esc index is configured as a can_throttle esc
 *
 * Read only function
 */
bool
UavcanHwescController::is_can_throttle(uint8_t esc_index) const
{
	if (esc_index >= MAX_ACTUATORS) {
		return false;
	}

	if (_param_handles[esc_index] == PARAM_INVALID) {
		return false;
	}

	int32_t fn = 0;
	if (param_get(_param_handles[esc_index], &fn) == 0 && fn > 0) {
		return true;
	}

	return false;
}

/**
 * Map and Assign all ESCs Once AFTER init.
 *
 * Reset previous mapping data
 *
 * Set configured ESCs on ESCStatus can_throttle flag according to _param_handles[]
 *
 * Call GetEscID service client
 */
void
UavcanHwescController::map_and_assign_all_once()
{
	const hrt_abstime now = hrt_absolute_time();

	// Don't reinit if already in progress or not requested
	if (_mapping_in_progress || !_reinit_requested || _armed) {
		return;
	}

	// Don't reinit if last attempt was too recent
	if (hrt_elapsed_time(&_last_esc_id_query) < GET_ESCID_COOLDOWN_US) {
		return;
	}

	_can_control_ready = false;
	_telem_ok = false;

	// pre-mapping uses user configuration before get esc id responses
	// how many escs are expected according to configuration and how many of them are can_throttle
	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		ESCStatus &esc = _esc_data[i];

		// This function might be requested in runtime, reset previous mapping data
		// Especially node id, since get esc id responses may assume duplicate
		esc.node_id = kInvalidNID;
		esc.last_seen_us = 0;
		esc.msg1_received = false;
		esc.msg2_received = false;
		esc.active = false;

		esc.can_throttle = is_can_throttle(i);				// Is rotor configured as a HWESC can throttle by UAVCAN_EC_FUNCx and UAVCAN_HWESC_PROTO params.
		esc.is_expected = esc.can_throttle || (i < _rotor_count);	// Is rotor expected according to either can_throttle or rotor count -> based on Mixing Class and UAVCAN_HWESC_PROTO
		esc.duplicate_mapping = false;
		esc.timestamp_get_esc_id = 0;					// This is checked inside finalize_mapping_window(), so reset it here.

	}

	// call get esc id service
	int res = get_esc_id_req(); // service call failed due to CAN driver issues
	if (res < 0) {
		if (res != -EAGAIN) {
			error_handler(HWESC_MAP_FAILED, 0, res);
		}
		return;
	}

	// service call ok, update last query time
	_last_esc_id_query = hrt_absolute_time();

	// notify mapping in progress
	_mapping_in_progress = true;

	// deadline for all get esc id responses is 0.2 s from now.
	_mapping_deadline_us = now + MAPPING_COLLECTION_WINDOW_US;
}

/**
 * Finalize the mapping window after collecting GetEscID responses
 *
 * Check for duplicate mappings
 *
 * Check for expected escs that did not respond
 *
 * Clear reinit request flag if mapping ok, otherwise set it to try again later
 */
void
UavcanHwescController::finalize_mapping_window()
{
	// Don't proceed if mapping is not in progress
	if (!_mapping_in_progress) {
		return;
	}

	_mapping_in_progress = false;
	_mapping_deadline_us = 0;

	bool mapping_ok = true;
	_telem_ok = true;

	// check get esc id responses against flags frosm the pre-mapping
	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		ESCStatus &esc = _esc_data[i];

		// check for duplicate mappings
		// this is a configuration error
		if (esc.duplicate_mapping) {
			_telem_ok = false;
			mapping_ok = false;
		}

		// check for expected escs that did not respond
		const bool responded = esc.timestamp_get_esc_id != 0;

		// for can throttle escs -> drone should not arm if mapping not ok
		if (esc.can_throttle && !responded) {
			error_handler(HWESC_MISSING_ESC, i, -1);
			mapping_ok = false;
		}
		// for non-can throttle escs -> warn only
		else if (esc.is_expected && !responded) {
			error_handler(HWESC_MISSING_ESC_WARN, i, -1);
		}
	}

	// drone can be armed if mapping_ok. note that we set can_control_ready based on
	// can_throttle && responded only. PWM motors are irrelevant.
	_can_control_ready = mapping_ok;

	// Setting _reinit_requested flag here is safe. Drone should already be disarmed in order to get here.
	// If drone is disarmed, it is safe to try mapping again later if not ok

	// If OK, clear reinit request flag
	// If not OK, set reinit request flag to try again later
	_reinit_requested = !mapping_ok;
}

bool
UavcanHwescController::has_duplicate_node_id(uint8_t nid, uint8_t exclude_idx) const
{
	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		if (i == exclude_idx) {
			continue;
		}

		const ESCStatus &esc = _esc_data[i];

		if (esc.node_id == nid) {
			return true;
		}
	}

	return false;
}

void
UavcanHwescController::update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned total_outputs)
{
	if (!_can_control_ready) {
		return;
	}
	// TODO: configurable rate limit
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_CMD_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	// output either 4 or 8 channels depending on max configured.

	// First fetch how many channels are configured via UAVCAN_EC_FUNCx

	if (_uavcan_rotor_count == 0) {
		return; // nothing configured, don't spam the bus
	}

	int required_len = (_uavcan_rotor_count <= 4) ? 4 : 8;

	if (required_len > MAX_ACTUATORS) {
		required_len = MAX_ACTUATORS;
	}

	com::hobbywing::esc::RawCommand msg = {};

	auto clamp_hw = [](int v) {
		// HW ESCs don't use reverse; keep non-negative and clamp to int14
		if (v < 0)   v = 0;
		if (v > INT14_MAX) v = INT14_MAX;
		return v;
	};

	for (int i = 0; i < required_len; i++) {
		int v = 0;

		if (i < (int)total_outputs) {
			v = static_cast<int>(outputs[i]);
		}

		msg.command.push_back(clamp_hw(v));
	}

	msg.command.resize(required_len);
	_uavcan_pub_raw_cmd.broadcast(msg);
}

void
UavcanHwescController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

/**
 * Query all nodes for GetMaintenanceInformation
 *
 * Option	 	= 0x00	(data field to get maintenance info)
 * Destination Node ID 	= 0x00 	(broadcast to all nodes)
 *
 * @return OK on success, negative errno on failure
 */
//int
//UavcanHwescController::get_maintenance_info_req()
//{
//	com::hobbywing::esc::GetMaintenanceInformation::Request req;
//	req.option = 0x00; 					// data field to get maintenance info
//	return _get_maintenance_info_client.call(0, req);
//}

//void
//UavcanHwescController::get_maintenance_info_cb(const uavcan::ServiceCallResult<com::hobbywing::esc::GetMaintenanceInformation> &result)
//{
//	// Check if the response, node_id etc. are valid
//	if (!result.isSuccessful()) return;
//	const uint8_t nid = result.getCallID().server_node_id.get();
//	if (nid == 0 || nid >= 0x7E) return; // skip reserved/invalid
//
//	const uint8_t idx = fetch_node_esc_index(nid);
//	if (idx == kInvalidIdx) return;
//
//	auto &esc = _esc_data[idx];
//
//	esc.life_cycle_min = result.getResponse().total_rotation_time_min;
//	esc.latest_cycle_min = result.getResponse().time_since_maintenance_min; // TODO: 24 bit CAN payload data to uint32_t fix
//}

/**
 * Periodic update function
 *
 * - Query all nodes for GetEscID every designated interval
 * - Publish ESC status uORB topic every designated interval
 * - Check for message timeouts and update active flags upon various timeouts
 *
*/
void
UavcanHwescController::periodic_update(const uavcan::TimerEvent &) {
       	const hrt_abstime now = hrt_absolute_time();

	// Check if drone is armed globally
	if (_actuator_armed_sub.updated()) {
		actuator_armed_s actuator_armed;
		if (_actuator_armed_sub.copy(&actuator_armed)) {
			if (actuator_armed.armed != _armed) {
				_armed = actuator_armed.armed;
				PX4_INFO("Actuator armed state changed: %s", _armed ? "ARMED" : "DISARMED");
			}
		}
	}

	// If reinit requested and not already in progress, start process
	if (_reinit_requested && !_mapping_in_progress && !_armed) {
		map_and_assign_all_once();
	}

	// We require to call finalize_mapping_window() here as well.
	// Even though it is called in get_esc_id_cb() when all expected escs are mapped.
	// Because, if the mapping window deadline passed and the last ESC don't reply, try finalize mapping after deadline.
	// This is to avoid waiting indefinitely if some expected escs are missing.
	// finalize_mapping_window() will not be called if mapping is not in progress.
	if (_mapping_in_progress && _mapping_deadline_us != 0 && now >= _mapping_deadline_us) {
		finalize_mapping_window();
	}

       	check_timeouts();			// Check for message timeouts and update active flags

	maybe_publish_esc_status();		// Publish esc_status uORB
	//maybe_publish_esc_hw_status();	// Publish surplus esc data
}

/**
 * Check for message timeouts and update active flags
 *
 * Only job is to update active flags based on timeouts
 * MSG1 timeout drives active flag
 * MSG2 timeout is informational only
 */
void
UavcanHwescController::check_timeouts()
{
	const hrt_abstime now = hrt_absolute_time();

	// Rate limit the scan
	if (_last_timeout_check != 0 && (now - _last_timeout_check) < CHECK_TIMEOUTS_MIN_US) {
		return;
	}

	const bool in_boot_grace = (now - _boot_time_us) < BOOT_GRACE_US;

	if (in_boot_grace) {
		// During boot grace period, don't mark anything inactive

		// but also don't forget to assign last timeout check time to avoid polling too often
		_last_timeout_check = now;
		return;
	}

	_last_timeout_check = now;

	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		auto &esc = _esc_data[i];

		// --- Stale flags (these don't decide 'active', just hygiene) ---
		if (esc.msg1_received && (now - esc.timestamp_msg1) > TIMEOUT_MSG1_US) {
		esc.msg1_received = false;
		}
		if (esc.msg2_received && (now - esc.timestamp_msg2) > TIMEOUT_MSG2_US) {
		esc.msg2_received = false;
		}
		//if (esc.msg3_received && (now - esc.timestamp_msg3) > TIMEOUT_MSG3_US) {
		//    esc.msg3_received = false;
		//}

		// --- Heartbeat (online) policy: MSG1 drives 'active' ---
		const bool msg1_fresh = (esc.timestamp_msg1 != 0) && ((now - esc.timestamp_msg1) <= TIMEOUT_MSG1_US);

		if (!msg1_fresh) {
		if (esc.active) {
			// Mark inactive
			esc.active = false;
			// Transition logging
			PX4_WARN("ESC %d (node %u) inactive (MSG1 timeout)", i, esc.node_id);
			error_handler(HWESC_MSG_TIMEOUT, i, -1);
		}
		} else {
		if (!esc.active) {
			// Mark active
			esc.active = true;
			// Transition logging
			PX4_INFO("ESC %d (node %u) active (MSG1 fresh)", i, esc.node_id);
		}
		}
	}
}

/**
 * Query all nodes for GetEscID as broadcast frame.
 *
 * payload is 3 bytes long, but only first byte is used.
 *
 * payload[0]	 	= 0x00	(data field to get ESC ID info)
 *
 * @return OK on success, negative errno on failure
 */
int
UavcanHwescController::get_esc_id_req()
{
	if (hrt_elapsed_time(&_last_esc_id_query) < GET_ESCID_COOLDOWN_US) {
		return -EAGAIN;
	}

	com::hobbywing::esc::GetEscID req{};
	req.payload[0] = 0x00;

	const int res = _pub_get_esc_id.broadcast(req);
	if (res >= 0) {
		_last_esc_id_query = hrt_absolute_time();
	}
	return res;
}

/**
 * Handle Service Response for GetEscID
 *
 * Response frame contains throttle channel (1-8) - maps to esc index (0-7)
 * Also, source node ID is extracted to match esc index.
 *
*/
void
UavcanHwescController::get_esc_id_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &msg) {
	// get node id and esc index from response. this is a received data structure with uint8_t payload[3] response
	// first byte is node id, second byte is throttle channel. third byte is tail byte.

	// received node id is checked if invalid.
    	const uint8_t nid = msg.getSrcNodeID().get();
    	if (nid == 0 || nid >= 0x7E) return; 	// TODO: Replace with a function that checks for valid node IDs and handle errors

	// received throttle channel is checked if invalid, then esc index is retrieved.

    	const uint8_t idx = throttle_ch2esc_index(msg.payload[1]);
    	if (idx == kInvalidIdx) {
		error_handler(HWESC_INVALID_INDEX, nid, -1);
        	// PX4_WARN("GetEscID bad channel from node %u", nid);
		return;
    	}

    	ESCStatus &esc = _esc_data[idx];

	// Duplicate Mappings - TYPE 0: banned node ids
	// Banned node ids are some specific IDs that are reserved for other nodes.
	if (is_banned_node(nid)) {
		// For this case we also set duplicate mapping.
		esc.duplicate_mapping = true;
		error_handler(HWESC_INVALID_ADDR, nid, -1);
		return;
	}

	// Duplicate mappings - TYPE I: two escs claiming the same esc index.
	// To check for duplicate mappings, check if the esc index already has a node id assigned
	// and if the assigned node id is different than the newly received node id.
	if (esc.node_id != kInvalidNID && esc.node_id != nid) {
		if (!esc.duplicate_mapping) {
			esc.duplicate_mapping = true;
			error_handler(HWESC_DUPLICATE_INDEX, idx, -1);
		}
		return;
	}
	// Duplicate mappings - TYPE II: two escs claiming the same node id.
	if (has_duplicate_node_id(nid, idx)) {
		if (!esc.duplicate_mapping) {
			esc.duplicate_mapping = true;
			error_handler(HWESC_DUPLICATE_ADDR, idx, -1);
		}
		return;
	}

	// Assign node id to esc index
	// Set other flags and timestamps
	esc.node_id     = nid;
	esc.is_hobbywing = true; // might need it later, will assign it regardless
	esc.active      = true; // active upon mapping, will be updated by timeouts
	esc.last_seen_us = hrt_absolute_time();
	esc.duplicate_mapping = false;
	esc.timestamp_get_esc_id = hrt_absolute_time();

	// If mapping in progress, check if all expected escs are mapped and finalize if so
	if (_mapping_in_progress) {
		bool all_expected_mapped = true;

		for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
			// If there are still escs expected but not yet mapped, continue waiting.
			if (_esc_data[i].is_expected && _esc_data[i].timestamp_get_esc_id == 0) {
				all_expected_mapped = false;
				// break and wait for next callback
				break;
			}
		}

		if (all_expected_mapped) {
			// Move to finalize mapping if all expected escs are mapped
			finalize_mapping_window();
		}
	}
}

/**
 * Handle StatusMsg1
 *
 */
void
UavcanHwescController::status_msg1_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg) {
	if (_mapping_in_progress) {
		// During mapping, ignore status messages
		return;
	}
	// If node id is banned, ignore message and log error
	int nid = msg.getSrcNodeID().get();
	if (is_banned_node(nid)) {
		error_handler(HWESC_INVALID_ADDR, nid, -1);
		return;
	}
	// Find esc index from node id
	const uint8_t idx = fetch_node_esc_index(nid);
	// Not found, might be a new esc, function returns invalid index and driver requests reinit.
	if (idx == kInvalidIdx) {
		error_handler(HWESC_UNDISC_ADDR, nid, -1);
		return;
	}

	// Get the ESC status reference
	auto &esc = _esc_data[idx];

	esc.rpm          = msg.rpm;
	esc.pwm          = msg.pwm;
	esc.status_flags = msg.status;

	// Manage timestamps and flags to help with timeouts and active status
	const uint64_t now = hrt_absolute_time();
	esc.timestamp_msg1 = now;
	esc.msg1_received  = true;
	esc.last_seen_us   = now;


}

/**
 * Handle StatusMsg2
 *
 */
void
UavcanHwescController::status_msg2_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg) {
	if (_mapping_in_progress) {
		// During mapping, ignore status messages
		return;
	}
	int nid = msg.getSrcNodeID().get();		// get node id
	if (is_banned_node(nid)) return;		// banned node id
	const uint8_t idx = fetch_node_esc_index(nid);	// fetch esc index from node id
	if (idx == kInvalidIdx) return;			// index not found

	auto &esc = _esc_data[idx];			// proceed with parsing the message

	esc.voltage_mv       = msg.input_voltage;
	esc.current_ma       = msg.current;
	esc.temperature_deg  = msg.temperature;

	const uint64_t now = hrt_absolute_time();
	esc.timestamp_msg2 = now;
	esc.msg2_received  = true;
	esc.last_seen_us   = now;
}


/**
 * Handle StatusMsg3
 *
 */
//void
//UavcanHwescController::status_msg3_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg) {
//	const uint8_t idx = fetch_node_esc_index(msg.getSrcNodeID().get());
//	if (idx == kInvalidIdx) return;
//	auto &esc = _esc_data[idx];
//
//	const uint64_t now = hrt_absolute_time();
//	esc.msg3_received = true;
//     	esc.timestamp_msg3 = now;
//	esc.last_seen_us   = now;
//}

/**
 * Convert HWESC status flags to esc_report_s failures
 *
 * HWESC status flags (16-bit):
 * Bit 0: Over Voltage
 * Bit 1: Under Voltage
 * Bit 2: Over Current
 * Bit 3: Positioning (Irrelevant for us)
 * Bit 4: Throttle Lost
 * Bit 5: Throttle Not Reset to Zero
 * Bit 6: MOS Overheating
 * Bit 7: Capacitor Overheating
 * Bit 8: Motor Stalled
 * Bit 9: MOS Open
 * Bit 10: MOS Short
 * Bit 11: Motor Disconnection
 * Bit 12: Opamp Failure
 * Bit 13: Communication Status (if bit is set abnormal)
 * Bit 14: Rotation Direction 0:CW 1:CCW (Irrelevant for us)
 * Bit 15: Throttle Source 0:CAN 1:Analog (Irrelevant for failure)
 *
 * uORB Side esc_report_s failures (16-bit):
 *
 * uint8 FAILURE_OVER_CURRENT = 0 		# (1 << 0)
 * uint8 FAILURE_OVER_VOLTAGE = 1 		# (1 << 1)
 * uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 	# (1 << 2)
 * uint8 FAILURE_OVER_RPM = 3			# (1 << 3)
 * uint8 FAILURE_INCONSISTENT_CMD = 4 		# (1 << 4)
 * uint8 FAILURE_MOTOR_STUCK = 5		# (1 << 5)
 * uint8 FAILURE_GENERIC = 6			# (1 << 6)
 * uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7	# (1 << 7)
 * uint8 FAILURE_WARN_ESC_TEMPERATURE = 8	# (1 << 8)
 * uint8 FAILURE_OVER_ESC_TEMPERATURE = 9	# (1 << 9)
 * uint8 ESC_FAILURE_COUNT = 10 		# Counter - keep it as last element!
 */
static inline uint16_t
statusflags_to_failures(uint16_t s)
{
// Maps Hobbywing 16-bit status to PX4 esc_report_s failures (bits 0..9).
// Bits 10..15 carry the count of set failure bits (0..63).
	uint16_t f = 0;

	// direct mappings
	if (s & (1u << 0))  f |= (1u << 1);  // Over Voltage -> OVER_VOLTAGE
	if (s & (1u << 2))  f |= (1u << 0);  // Over Current -> OVER_CURRENT
	if (s & (1u << 4))  f |= (1u << 4);  // Throttle Lost -> INCONSISTENT_CMD
	if (s & (1u << 5))  f |= (1u << 4);  // Throttle Not Zero -> INCONSISTENT_CMD
	if (s & (1u << 6))  f |= (1u << 9);  // MOS Overheat -> ESC_OVER_TEMP (critical)
	if (s & (1u << 7))  f |= (1u << 8);  // Capacitor Overheat -> ESC_WARN_TEMP (warn)
	if (s & (1u << 8))  f |= (1u << 5);  // Motor Stalled -> MOTOR_STUCK

	// group to GENERIC: undervolt, MOS open/short, motor disconnect, opamp fail, comm abnormal
	if (s & ((1u << 1) | (1u << 9) | (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13))) {
		f |= (1u << 6);                  // GENERIC
	}

	// pack count of failure bits 0..9 into 10..15
	#if defined(__GNUC__) || defined(__clang__)
	uint16_t cnt = __builtin_popcount(f & 0x03FFu);
	#else
	// portable popcount for 10 bits
	uint16_t x = (f & 0x03FFu);
	x = (x & 0x5555u) + ((x >> 1) & 0x5555u);
	x = (x & 0x3333u) + ((x >> 2) & 0x3333u);
	x = (x + (x >> 4)) & 0x0F0Fu;
	x = (x + (x >> 8)) & 0x001Fu;
	uint16_t cnt = x;
	#endif
	if (cnt > 63) cnt = 63;
	return f | (cnt << 10);
}

/**
 * Publish esc_hw_status uORB topic if any ESC is active
 *
 * This topic contains additional information not present in esc_status,
 * such as PWM, status flags, temperatures.
 *
*/
//void
//UavcanHwescController::maybe_publish_esc_hw_status()
//{
//	// Publish MSG3 data to esc_hw_status uORB topic whenever possible.
//	esc_hw_status_s hwesc{};
//	hwesc.timestamp = hrt_absolute_time();
//
//	// Reason to use any_active flag is to avoid publishing empty topics
//	// when no ESC is connected/active.
//	// This topic is not as critical as esc_status, so we can save some memory
//	// and CPU by not publishing useless data.
//	bool any_active = false;
//
//	// For each possible esc index, update the corresponding esc_report_s uORB
//	for (int i = 0; i < MAX_ACTUATORS; ++i) {
//		const ESCStatus &e = _esc_data[i];
//
//		// Consider ESC ready if MSG3 received (and not published yet) and active flag is set
//		const bool online  = e.msg3_received && e.active;
//
//		if (online) {
//			any_active = true;
//			hwesc.status_flags[i]     = e.status_flags;
//			hwesc.mosfet_temperature[i]  = e.mos_temp_deg;
//			hwesc.capacitor_temperature[i] = e.cap_temp_deg;
//			hwesc.motor_temperature[i]    = e.motor_temp_deg;
//		} else {
//			hwesc.status_flags[i]     = 0;
//			hwesc.mosfet_temperature[i]  = 0;
//			hwesc.capacitor_temperature[i] = 0;
//			hwesc.motor_temperature[i]    = 0;
//		}
//		// Clear edge flags for next cycle after publishing so don't report same data again
//		_esc_data[i].msg3_received = false;
//	}
//
//	if (any_active) {
//		_hwesc_status_pub.publish(hwesc);
//	}
//}

void
UavcanHwescController::maybe_publish_esc_status()
{
	if (_mapping_in_progress) {
		// During mapping, ignore status messages
		return;
	}
	// Publish MSG1 and MSG2 data to esc_status uORB topic whenever possible.
	esc_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	status.esc_online_flags = 0;
	status.esc_armed_flags  = 0;

	uint8_t online_count = 0;

	// For each possible esc index, update the corresponding esc_report_s uORB
	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		const ESCStatus &e = _esc_data[i];

		// Consider ESC ready if MSG1 and MSG2 received (and not published yet) and active flag is set
		const bool online  = e.msg1_received && e.msg2_received && e.active;

		auto &ref = status.esc[i];

		if (online) {
		++online_count;

		ref.timestamp       = status.timestamp;
		ref.esc_address     = e.node_id;
		ref.esc_voltage     = e.voltage_mv * 0.1f;
		ref.esc_current     = e.current_ma * 0.1f;
		ref.esc_temperature = e.temperature_deg;
		ref.esc_rpm         = e.rpm;
		ref.esc_power       = e.pwm/8191.0f;	// TODO: 0-8191 maps to 0-100% power - variable type mismatch, fix uint16_t to int8_t in uORB side.
		ref.failures	    = statusflags_to_failures(e.status_flags);
		ref.esc_errorcount  = 0;		// TODO: add error count if available

		status.esc_online_flags |= (1u << i);
		status.esc_armed_flags  |= (1u << i);	// TODO: decide how to manage armed flags for can escs apart from online only
		} else {
			// Not online.
		}
		// Clear edge flags for next cycle after publishing so don't report same data again.
		_esc_data[i].msg1_received = false;
		_esc_data[i].msg2_received = false;
	}

	status.esc_count = online_count;
	status.counter = _status_counter++;
	_esc_status_pub.publish(status);
}

uint8_t
UavcanHwescController::fetch_node_esc_index(uint8_t node_id) const
{
	// Search esc index for given node id
	for (int i = 0; i < MAX_ACTUATORS; i++) {
		if (_esc_data[i].node_id == node_id) {
			// Given node ID corresponds to esc index i
			return i;
		}
	}
	// Always return invalid idx if not found otherwise error handler might loop and give 0 to unknown node ids
	return kInvalidIdx;
}

bool
UavcanHwescController::is_banned_node(uint8_t node_id) const
{
    if (node_id < sizeof(banned_node_ids)) {
        return banned_node_ids[node_id];
    }
    return false;
}

int
UavcanHwescController::error_handler(uint8_t error_type, uint8_t val, int res)
{
	static uint8_t prev_error_type = HWESC_OK;
	static int error_count = 0;
	static hrt_abstime last_error_time = 0;

	const hrt_abstime now = hrt_absolute_time();   // <-- move to top

	// if res >= 0 it's informational; don't early-return OK because callers
	// may rely on side effects (like setting _reinit_requested)
	// We'll still proceed with switching on error_type.

	// simple 1 Hz rate limit for errors.
	if ((now - last_error_time) < 1'000'000 && error_type == prev_error_type) {
		return error_type;
	}
	last_error_time = now;

	prev_error_type = error_type;
	error_count++;

	bool try_reinit = false;

	switch (error_type) {
	case HWESC_MAP_FAILED:
	case HWESC_INVALID_INDEX:
	case HWESC_UNEXPECTED_INDEX:
	case HWESC_MISSING_ESC:
		PX4_ERR("MISSING ESC! DON'T ARM");
		try_reinit = true;
		break;
	case HWESC_DUPLICATE_INDEX:
	case HWESC_DUPLICATE_ADDR:
	case HWESC_MSG_TIMEOUT:
	case HWESC_INVALID_ADDR:
	case HWESC_MAP_TIMEOUT:
		try_reinit = true;
		break;

	case HWESC_MISSING_ESC_WARN:
	case HWESC_UNDISC_ADDR:
	case HWESC_OK:
	default:
		PX4_ERR("HWESC error: %u (val=%u, res=%d)", error_type, val, res); // log all errors.
		break;
	}

	if (try_reinit) {
		if (_armed) {
			PX4_WARN("Cannot reinitialize Hobbywing ESCs while armed");
		} else {
			_reinit_requested = true;
		}
	}

	return OK; // TODO: improve as a stateful handler: Return OK when you handled it , and a negative errno only if you couldn’t.
}

// later on add logic for maintenance info. void esc_maintenance_res_cb(const uavcan::ServiceCallResult<com::hobbywing::esc::GetMaintenanceInformation> &msg) { }
