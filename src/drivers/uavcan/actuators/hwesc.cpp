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
#include <string.h>
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

	request_remap();
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

void
UavcanHwescController::start_mapping()
{
	if (_mapping_in_progress || _armed || !_need_remap) {
		return;
	}

	if (hrt_elapsed_time(&_last_esc_id_query_us) < GET_ESCID_COOLDOWN_US) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	_mapping_in_progress = true;
	_last_remap_try_us = now;

	if (++_current_map_attempt == 0) {
		_current_map_attempt = 1;
	}

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		ESCStatus &esc = _esc_data[i];
		esc.can_throttle = is_can_throttle(i);
		esc.is_expected = esc.can_throttle || (i < _rotor_count);
		esc.pending_node_id = kInvalidNID;
		esc.duplicate_index = false;
		esc.duplicate_addr = false;
		esc.invalid_node_id = false;
		esc.timestamp_get_esc_id = 0;
		esc.last_map_attempt = 0;
	}

	const int res = get_esc_id_req();

	if (res < 0) {
		_mapping_in_progress = false;
		_mapping_deadline_us = 0;

		if (res != -EAGAIN) {
			PX4_ERR("GetEscID request failed (%d)", res);
			set_error(HwescErrorBit::MapFailed, true, true);
			request_remap();
			apply_mapping_failure_backoff();
		}

		_last_remap_try_us = now;
		return;
	}

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
UavcanHwescController::finalize_mapping()
{
	if (!_mapping_in_progress) {
		return;
	}

	_mapping_in_progress = false;
	_mapping_deadline_us = 0;

	const uint16_t attempt = _current_map_attempt;
	const hrt_abstime now = hrt_absolute_time();

	uint32_t new_ctrl = _err_ctrl & ~MAPPING_ERROR_MASK;
	uint32_t new_telem = _err_telem & ~MAPPING_ERROR_MASK;

	bool any_response = false;

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		ESCStatus &esc = _esc_data[i];

		const bool responded = (esc.last_map_attempt == attempt) && (esc.pending_node_id != kInvalidNID);

		if (responded) {
			any_response = true;
			esc.node_id = esc.pending_node_id;
			esc.pending_node_id = kInvalidNID;
			esc.timestamp_get_esc_id = now;
			esc.last_map_attempt = attempt;
			continue;
		}

			if (esc.duplicate_index) {
				new_ctrl |= bit_mask(HwescErrorBit::DupIndex);
				new_telem |= bit_mask(HwescErrorBit::DupIndex);
			}

			if (esc.duplicate_addr) {
				new_ctrl |= bit_mask(HwescErrorBit::DupAddr);
				new_telem |= bit_mask(HwescErrorBit::DupAddr);
			}

			if (esc.invalid_node_id) {
				new_ctrl |= bit_mask(HwescErrorBit::InvalidAddr);
				new_telem |= bit_mask(HwescErrorBit::InvalidAddr);
			}

			if (esc.can_throttle) {
				new_ctrl |= bit_mask(HwescErrorBit::Missing);
				new_telem |= bit_mask(HwescErrorBit::Missing);
				PX4_ERR("HWESC mapping: CAN ESC index %u missing", i);
			} else if (esc.is_expected) {
				new_telem |= bit_mask(HwescErrorBit::Missing);
				PX4_WARN("HWESC mapping: telemetry missing for ESC index %u", i);
			}

			esc.pending_node_id = kInvalidNID;
			esc.last_map_attempt = 0;
	}

		if (!any_response) {
			new_ctrl |= bit_mask(HwescErrorBit::MapNoResp);
			new_telem |= bit_mask(HwescErrorBit::MapNoResp);
			PX4_WARN("HWESC mapping: no GetEscID responses");
		}

	_err_ctrl = new_ctrl;
	_err_telem = new_telem;
	update_readiness_flags();

	const bool control_ok = (_err_ctrl & CONTROL_BLOCKING_MASK) == 0;

	if (control_ok) {
		_need_remap = false;
		reset_remap_backoff();
	} else {
		request_remap();
		apply_mapping_failure_backoff();
	}
}

int
UavcanHwescController::find_duplicate_node_id(uint8_t nid, uint8_t exclude_idx) const
{
	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		if (i == exclude_idx) {
			continue;
		}

		const ESCStatus &esc = _esc_data[i];
		const uint8_t candidate = (esc.pending_node_id != kInvalidNID) ? esc.pending_node_id : esc.node_id;

		if (candidate == nid) {
			return i;
		}
	}

	return -1;
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
				handle_arm_state_change(actuator_armed.armed);
			}
		}
	}

	attempt_remap(now);

	// We require to call finalize_mapping() here as well.
	// Even though it is called in get_esc_id_cb() when all expected escs are mapped.
	// Because, if the mapping window deadline passed and the last ESC don't reply, try finalize mapping after deadline.
	// This is to avoid waiting indefinitely if some expected escs are missing.
	// finalize_mapping() will not be called if mapping is not in progress.
	if (_mapping_in_progress && _mapping_deadline_us != 0 && now >= _mapping_deadline_us) {
		finalize_mapping();
	}

       	check_timeouts();			// Check for message timeouts and update active flags
	evaluate_all_dark(now);

	if (is_ready_to_publish()) {
		publish_esc_status();		// Publish esc_status uORB
	}
	//maybe_publish_esc_hw_status();	// Publish surplus esc data
}

/**
 * Check for message timeouts and update active flags
 *
 * MSG1 timeout drives active flag
 * MSG2 timeout is informational only
 */
void
UavcanHwescController::check_timeouts()
{
	const hrt_abstime now = hrt_absolute_time();

	// Rate limit the scan
	if (_last_timeout_check_us != 0 && (now - _last_timeout_check_us) < CHECK_TIMEOUTS_MIN_US) {
		return;
	}

	const bool in_boot_grace = (now - _boot_time_us) < BOOT_GRACE_US;

	if (in_boot_grace) {
		// During boot grace period, don't mark anything inactive

		// but also don't forget to assign last timeout check time to avoid polling too often
		_last_timeout_check_us = now;
		return;
	}

	_last_timeout_check_us = now;

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
	if (hrt_elapsed_time(&_last_esc_id_query_us) < GET_ESCID_COOLDOWN_US) {
		return -EAGAIN;
	}

	com::hobbywing::esc::GetEscID req{};
	req.payload.push_back(0x00);

	const int res = _pub_get_esc_id.broadcast(req);
	if (res >= 0) {
		_last_esc_id_query_us = hrt_absolute_time();
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
    	if (nid == 0 || nid >= 0x7E) {
		PX4_ERR("HWESC mapping: invalid node id %u", nid);
		set_error(HwescErrorBit::InvalidAddr, true, true);
		request_remap();
		return;
	}

	// received throttle channel is checked if invalid, then esc index is retrieved.

    	const uint8_t idx = throttle_ch2esc_index(msg.payload[1]);
    	if (idx == kInvalidIdx) {
		PX4_ERR("HWESC mapping: invalid throttle channel %u from node %u", msg.payload[1], nid);
		set_error(HwescErrorBit::InvalidIndex, true, true);
		request_remap();
		return;
    	}

    	ESCStatus &esc = _esc_data[idx];

	// Duplicate Mappings - TYPE 0: banned node ids
	// Banned node ids are some specific IDs that are reserved for other nodes.
	if (is_banned_node(nid)) {
		// For this case we also set duplicate mapping.
		esc.invalid_node_id = true;
		PX4_ERR("HWESC mapping: node %u is banned (idx %u)", nid, idx);
		set_error(HwescErrorBit::InvalidAddr, true, true);
		request_remap();
		return;
	}

	// Duplicate mappings - TYPE I: two escs claiming the same esc index.
	// To check for duplicate mappings, check if the esc index already has a node id assigned
	// and if the assigned node id is different than the newly received node id.
	if (esc.pending_node_id != kInvalidNID && esc.pending_node_id != nid) {
		esc.duplicate_index = true;
		PX4_ERR("HWESC mapping: duplicate index %u for node %u (existing node %u)", idx, nid, esc.pending_node_id);
		set_error(HwescErrorBit::DupIndex, true, true);
		request_remap();
		return;
	}
	// Duplicate mappings - TYPE II: two escs claiming the same node id.
	const int dup_idx = find_duplicate_node_id(nid, idx);

	if (dup_idx >= 0) {
		ESCStatus &dup = _esc_data[dup_idx];
		esc.duplicate_addr = true;
		dup.duplicate_addr = true;
		PX4_ERR("HWESC mapping: duplicate node id %u between idx %u and %d", nid, idx, dup_idx);
		set_error(HwescErrorBit::DupAddr, true, true);
		request_remap();
		return;
	}

	// Unexpected index beyond configuration (captures configuration errors)
	if (!esc.is_expected) {
		PX4_WARN("HWESC mapping: unexpected index %u reported by node %u", idx, nid);
		set_error(HwescErrorBit::UnexpectedIndex, true, true);
		request_remap();
	}

	esc.pending_node_id = nid;
	esc.invalid_node_id = false;
	esc.timestamp_get_esc_id = hrt_absolute_time();
	esc.last_map_attempt = _current_map_attempt;
	esc.last_seen_us = esc.timestamp_get_esc_id;
	esc.active = true;
	esc.is_hobbywing = true;

	if (_mapping_in_progress) {
		bool all_expected_mapped = true;

		for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
			const ESCStatus &candidate = _esc_data[i];

			if (!candidate.is_expected) {
				continue;
			}

			if (candidate.duplicate_index || candidate.duplicate_addr || candidate.invalid_node_id) {
				all_expected_mapped = false;
				break;
			}

			if (candidate.pending_node_id == kInvalidNID) {
				all_expected_mapped = false;
				break;
			}
		}

		if (all_expected_mapped) {
			finalize_mapping();
		}
	}
}

void
UavcanHwescController::status_msg1_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg) {
	if (_mapping_in_progress) {
		// During mapping, ignore status messages
		return;
	}

	const uint8_t nid = msg.getSrcNodeID().get();

	if (is_banned_node(nid)) {
		if (!_unknown_addr_logged[nid]) {
			PX4_ERR("HWESC runtime: banned node %u", nid);
			_unknown_addr_logged[nid] = true;
		}

		set_error(HwescErrorBit::InvalidAddr, true, true);
		request_remap();
		return;
	}

	const uint8_t idx = fetch_node_esc_index(nid);

	if (idx == kInvalidIdx) {
		if (!_unknown_addr_logged[nid]) {
			PX4_WARN("HWESC runtime: unknown node %u (%s)", nid, _armed ? "armed" : "disarmed");
			_unknown_addr_logged[nid] = true;
		}

		if (_armed) {
			set_error(HwescErrorBit::UndiscAddr, false, true);
		} else {
			set_error(HwescErrorBit::UndiscAddr, true, true);
		}

		request_remap();
		return;
	}

	// Get the ESC status reference
	auto &esc = _esc_data[idx];
	_unknown_addr_logged[nid] = false;

	esc.rpm          = msg.rpm;
	esc.pwm          = msg.pwm;
	esc.status_flags = msg.status;

	// Manage timestamps and flags to help with timeouts and active status
	const uint64_t now = hrt_absolute_time();
	esc.timestamp_msg1 = now;
	esc.msg1_received  = true;
	esc.last_seen_us   = now;
	esc.active = true;
}

void
UavcanHwescController::status_msg2_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg) {
	if (_mapping_in_progress) {
		// During mapping, ignore status messages
		return;
	}
	const uint8_t nid = msg.getSrcNodeID().get();		// get node id

	if (is_banned_node(nid)) {
		if (!_unknown_addr_logged[nid]) {
			PX4_ERR("HWESC runtime: banned node %u (MSG2)", nid);
			_unknown_addr_logged[nid] = true;
		}
		set_error(HwescErrorBit::InvalidAddr, true, true);
		request_remap();
		return;
	}

	const uint8_t idx = fetch_node_esc_index(nid);	// fetch esc index from node id

	if (idx == kInvalidIdx) {
		if (!_unknown_addr_logged[nid]) {
			PX4_WARN("HWESC runtime: unknown node %u in MSG2 (%s)", nid, _armed ? "armed" : "disarmed");
			_unknown_addr_logged[nid] = true;
		}

		if (_armed) {
			set_error(HwescErrorBit::UndiscAddr, false, true);
		} else {
			set_error(HwescErrorBit::UndiscAddr, true, true);
		}

		request_remap();
		return;
	}

	auto &esc = _esc_data[idx];			// proceed with parsing the message
	_unknown_addr_logged[nid] = false;

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
 * Convert Hobbywing 16-bit status flags to PX4 esc_report_s failures field.
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

bool
UavcanHwescController::is_ready_to_publish() const
{
    if (_mapping_in_progress) {
        return false;
    }

    bool any_active = false;

    for (int i = 0; i < MAX_ACTUATORS; ++i) {
        const ESCStatus &e = _esc_data[i];

        if (!e.active) {
            continue;
        }

        any_active = true;

        // require both vendor frames for active ESCs
        if (!e.msg1_received || !e.msg2_received) {
            return false;
        }
    }

    return any_active; // avoid publishing empty frames
}

void
UavcanHwescController::publish_esc_status()
{
	// Publish MSG1 and MSG2 data to esc_status uORB topic whenever possible.
	esc_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	status.esc_online_flags = 0;
	status.esc_armed_flags  = 0;

	uint8_t online_count = 0;

	// For each active esc index, update the corresponding esc_report_s uORB
	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		const ESCStatus &e = _esc_data[i];

		// ESC readiness is checked by is_ready_to_publish() before calling this function.
		// We only sweep through active escs and publish their data.
		auto &ref = status.esc[i];

		if (e.active) {
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

			// Clear edge flags for next cycle so don't report same data again.
			_esc_data[i].msg1_received = false;
			_esc_data[i].msg2_received = false;
		} else {
			// Not online.
		}
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

bool
UavcanHwescController::is_mapping_error(HwescErrorBit bit) const
{
	switch (bit) {
	case HwescErrorBit::DupIndex:
	case HwescErrorBit::DupAddr:
	case HwescErrorBit::InvalidAddr:
	case HwescErrorBit::Missing:
	case HwescErrorBit::UnexpectedIndex:
	case HwescErrorBit::InvalidIndex:
	case HwescErrorBit::MapFailed:
	case HwescErrorBit::MapNoResp:
	case HwescErrorBit::UndiscAddr:
		return true;

	default:
		break;
	}

	return false;
}

void
UavcanHwescController::set_error(HwescErrorBit bit, bool ctrl, bool telem)
{
	const uint32_t mask = bit_mask(bit);

	if (ctrl) {
		_err_ctrl |= mask;
	}

	if (telem) {
		_err_telem |= mask;
	}

	if (is_mapping_error(bit)) {
		request_remap();
	}

	update_readiness_flags();
}

void
UavcanHwescController::clear_error_mask(uint32_t mask, bool ctrl, bool telem)
{
	if (ctrl) {
		_err_ctrl &= ~mask;
	}

	if (telem) {
		_err_telem &= ~mask;
	}

	update_readiness_flags();
}

void
UavcanHwescController::clear_mapping_error_bits()
{
	clear_error_mask(MAPPING_ERROR_MASK, true, true);
}

void
UavcanHwescController::update_readiness_flags()
{
	_can_control_ready = (_err_ctrl & CONTROL_BLOCKING_MASK) == 0;
	_telem_ok = (_err_telem & TELEMETRY_BLOCKING_MASK) == 0;
}

void
UavcanHwescController::request_remap()
{
	if (!_need_remap) {
		_need_remap = true;
		reset_remap_backoff();
	}
}

void
UavcanHwescController::apply_mapping_failure_backoff()
{
	if (_remap_backoff_us < REMAP_BACKOFF_MAX_US) {
		uint64_t next = _remap_backoff_us * 2;

		if (next > REMAP_BACKOFF_MAX_US) {
			next = REMAP_BACKOFF_MAX_US;
		}

		_remap_backoff_us = next;
	}

	if (_remap_attempts < UINT32_MAX) {
		++_remap_attempts;
	}
}

void
UavcanHwescController::reset_remap_backoff()
{
	_remap_attempts = 0;
	_remap_backoff_us = REMAP_BACKOFF_BASE_US;
	_last_remap_try_us = 0;
}

void
UavcanHwescController::attempt_remap(hrt_abstime now)
{
	if (!_need_remap || _mapping_in_progress || _armed) {
		return;
	}

	if (_remap_attempts > 0 && _last_remap_try_us != 0) {
		if ((now - _last_remap_try_us) < _remap_backoff_us) {
			return;
		}
	}

	if (hrt_elapsed_time(&_last_esc_id_query_us) < GET_ESCID_COOLDOWN_US) {
		return;
	}

	start_mapping();
}

void
UavcanHwescController::evaluate_all_dark(hrt_abstime now)
{
	if (_armed) {
		_all_dark_since_us = 0;
		return;
	}

	bool any_mapped = false;
	bool any_active = false;

	for (int i = 0; i < MAX_ACTUATORS; ++i) {
		const ESCStatus &esc = _esc_data[i];

		if (esc.node_id != kInvalidNID) {
			any_mapped = true;

			if (esc.active) {
				any_active = true;
				break;
			}
		}
	}

	if (!any_mapped) {
		_all_dark_since_us = 0;
		return;
	}

	if (!any_active) {
		if (_all_dark_since_us == 0) {
			_all_dark_since_us = now;
		} else if ((now - _all_dark_since_us) > ALL_DARK_TO_REMAP_US) {
			request_remap();
			_all_dark_since_us = now;
		}
	} else {
		_all_dark_since_us = 0;
	}
}

void
UavcanHwescController::handle_arm_state_change(bool armed)
{
	_armed = armed;
	PX4_INFO("Actuator armed state changed: %s", _armed ? "ARMED" : "DISARMED");

	if (!_armed) {
		memset(_unknown_addr_logged, 0, sizeof(_unknown_addr_logged));
		_all_dark_since_us = 0;
		_last_remap_try_us = 0;
	} else {
		_all_dark_since_us = 0;
	}
}
