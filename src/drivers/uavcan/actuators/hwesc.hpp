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
 * Hobbywing ESC UAVCAN driver for PX4-Autopilot.
 *
 * @file hwesc.hpp
 *
 * UAVCAN <--> MixerModule bridge for Hobbywing ESC commands:
 *     com.hobbywing.esc.RawCommand
 *
 * UAVCAN <--> ORB bridge for HW ESC messages:
 *     com.hobbywing.esc.GetEscID
 *     com.hobbywing.esc.StatusMsg1
 *     com.hobbywing.esc.StatusMsg2
 *
 * @author Caglar Yilmaz <yilmaz.caglar@tubitak.gov.tr>
 */

#pragma once

#include <uavcan/uavcan.hpp>

#include <com/hobbywing/esc/RawCommand.hpp>
#include <com/hobbywing/esc/StatusMsg1.hpp>
#include <com/hobbywing/esc/StatusMsg2.hpp>
#include <com/hobbywing/esc/GetEscID.hpp>

#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

#include <uORB/topics/esc_status.h>

#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <parameters/param.h>

#include <px4_platform_common/module_params.h>

class UavcanHwescController : public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;	// < How many ESCs are expected maximum
	static constexpr unsigned MAX_CMD_RATE_HZ = 400;			// < max rate for command updates
	static constexpr unsigned MAX_MSG_RATE_HZ = 10;				// < max rate for telemetry updates
	static constexpr unsigned MAX_GET_RATE_HZ = 10;				// < max rate for GetEscID requests

	static_assert(com::hobbywing::esc::RawCommand::FieldTypes::command::MaxSize >= MAX_ACTUATORS, "Too many actuators");

	UavcanHwescController(uavcan::INode &node);
	~UavcanHwescController() = default;

	/**
	 * Initialize the UAVCAN ESC controller node
	 *
	 * @return 0 on success, negative error code on failure
	 */
	int init();

	/* Public Functions */

	/**
	 * Update actuator outputs.
	 *
	 * This function sends the updated actuator output values to the corresponding ESCs.
	 *
	 * @param outputs Array of actuator output values.
	 * @param total_outputs Total number of outputs in the array.
	 */
	void update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned total_outputs);

	esc_status_s &esc_status() { return _esc_status; }

	/**
	 * Sets the number of rotors and enable timer
	 *
	 * @param count Number of rotors from mixer
	 */
	void set_rotor_count(uint8_t count);

	static constexpr int INT14_MAX = 8191;	// maximum value for each ESC in RawCommand
	static constexpr int INT14_MIN = -8192; // not used for HWESC
	/**
	 * Get maximum output value for ESC command
	 *
	 * @return Maximum output value for ESC command
	*/
	static int max_output_value() { return INT14_MAX; }

private:
	/**
	 * Error Bits for HWESC Driver
	 *
	 * Each bit represents a specific error condition that can occur during the operation of the HWESC driver.
	 * These error bits are used to track and manage the state of the driver, allowing for appropriate responses.
	 */
	enum class HwescErrorBit : uint8_t {
		DupIndex = 0,		//< If more than one node mapped to same index
		DupAddr,		//< If more than one index mapped to same address
		InvalidAddr,		//< If an invalid or banned Node ID is detected during mapping
		Missing,		//< If an expected ESC is missing after mapping
		UnexpectedIndex,	//< If an unexpected index is detected during mapping
		InvalidIndex,		//< If an invalid index is detected during mapping
		MapFailed,		//< If mapping fails
		MapNoResp,		//< If no response is received during mapping
		UndiscAddr,		//< If an undiscovered address is detected during operation
		_count			//< Total number of error bits
	};

	static constexpr uint32_t CONTROL_BLOCKING_MASK =
		(1u << static_cast<uint8_t>(HwescErrorBit::DupIndex)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::DupAddr)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::InvalidAddr)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::Missing)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::UnexpectedIndex)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::InvalidIndex)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::MapFailed)) |
		(1u << static_cast<uint8_t>(HwescErrorBit::MapNoResp));

	static constexpr uint32_t TELEMETRY_BLOCKING_MASK = CONTROL_BLOCKING_MASK |
		(1u << static_cast<uint8_t>(HwescErrorBit::UndiscAddr));

	static constexpr uint32_t MAPPING_ERROR_MASK = TELEMETRY_BLOCKING_MASK;

	static constexpr uint32_t bit_mask(HwescErrorBit bit) { return 1u << static_cast<uint8_t>(bit); }

	struct ESCStatus {
		bool     is_expected{false};						// expected ESCs according to _rotor_count regardless of physical actuator link
		bool     is_hobbywing{false};						// might need it later, will assign it regardless
		bool     can_throttle{false};						// whether this esc index is configured to be controlled via CAN
		bool     duplicate_index{false};					// whether this esc index has duplicate index in mapping
		bool     duplicate_addr{false};						// whether this esc index has duplicate address in mapping
		bool     invalid_node_id{false};					// whether this esc index has invalid node id in mapping

		uint8_t  node_id{kInvalidNID};						// currently mapped node id
		uint8_t  pending_node_id{kInvalidNID};					// pending node id during mapping
		uint16_t rpm{0};
		uint16_t pwm{0};
		uint16_t status_flags{0};
		uint16_t voltage_mv{0};
		uint16_t current_ma{0};
		uint8_t  temperature_deg{0};
		// uint8_t  mos_temp_deg;
		// uint8_t  cap_temp_deg;
		// uint8_t  motor_temp_deg;

		// uint32_t life_cycle_min{};
		// uint32_t latest_cycle_min{};

		hrt_abstime timestamp_msg1{0};
		hrt_abstime timestamp_msg2{0};
		// hrt_abstime timestamp_msg3{0};
		hrt_abstime timestamp_get_esc_id{0};
		hrt_abstime last_seen_us{0};						// last time we received any message from this esc

		bool 	 msg1_received{false};
		bool 	 msg2_received{false};
		// bool  msg3_received{false};
		bool	 active{false};
		uint16_t last_map_attempt{0};						// last mapping attempt number
	};

	static constexpr uint8_t kInvalidIdx = 0xFF;   					// invalid esc index
	static constexpr uint8_t kInvalidNID = 0x00;   					// invalid node id

	static constexpr uint64_t ALL_DARK_TO_REMAP_US 		= 400'000;		// time since all escs dark to request remap
	static constexpr uint64_t REMAP_BACKOFF_BASE_US 	= 500'000;		// initial backoff for remap attempts
	static constexpr uint64_t REMAP_BACKOFF_MAX_US 		= 5'000'000;		// exponential backoff for remap attempts
	static constexpr uint64_t BOOT_GRACE_US            	= 2'000'000;  		// after boot/mapping, don't mark inactive too early
	static constexpr uint64_t TIMEOUT_MSG1_US          	= 700'000;  		// heartbeat (MSG1) timeout
	static constexpr uint64_t TIMEOUT_MSG2_US          	= 700'000;  		// slower, informational
	static constexpr uint64_t CHECK_TIMEOUTS_MIN_US    	= 100'000;    		// don't scan too often (10 Hz)
	static constexpr uint64_t MAPPING_COLLECTION_WINDOW_US 	= 500'000; 		// 500 ms window to gather GetEscID responses
	static constexpr uint64_t WARN_THROTTLE_INTERVAL_US    	= 1'000'000; 		// 1 second
	static constexpr uint64_t GET_ESCID_COOLDOWN_US = 1'000'000 / MAX_GET_RATE_HZ;	// cooldown between GetEscID requests

	hrt_abstime 	_all_dark_since_us{0};						// < time since all ESCs have been dark
	hrt_abstime 	_last_esc_id_query_us{0};					// < last time we queried for GetEscID
	hrt_abstime 	_boot_time_us{hrt_absolute_time()};				// < system time since boot
	hrt_abstime 	_last_timeout_check_us{0};					// < last time we checked for timeouts
	hrt_abstime 	_mapping_deadline_us{0};					// < deadline for mapping collection window
	hrt_abstime 	_last_remap_try_us{0};						// < last time we attempted remap
	hrt_abstime 	_last_esc_id_query_us{0};					// < last time we queried for GetEscID
	uint64_t 	_remap_backoff_us{REMAP_BACKOFF_BASE_US};			// < current backoff duration

	uint32_t 	_remap_attempts{0};						// < number of remap attempts
	uint16_t 	_current_map_attempt{0};					// < current mapping attempt number
	uint8_t		_rotor_count{0};						// < number of rotors from mixer
	uint8_t		_uavcan_rotor_count{0};						// < number of rotors controllable via uavcan
	bool 		_need_remap{false};						// < whether a remap is requested
	bool 		_mapping_in_progress{false};					// < whether we are in mapping process
	bool 		_armed{false};							// < whether the system is armed or not

	bool 		_can_control_ready{false};					// < whether the system is ready for CAN control (set to true if PWM regardless for now)

	bool 		_telem_ok{false};						// < whether telemetry is OK to publish

	uint32_t 	_err_ctrl{0};							// < error bitmasks for control
	uint32_t 	_err_telem{0};							// < error bitmasks for telemetry

	uint16_t 	_status_counter{0};						// < internal counter for esc status topic

	/**
	 * This is a 128 bit array to rate limit the user notification of unmapped or banned node IDs.
	 * When an unknown node ID is detected via MSG1 or MSG2:
	 * - Corresponding callback detects it and checks whether it is a banned node ID.
	 * - If so, it checks this array to see if a warning has already been logged for this node ID.
	 * - If not, it logs a warning and sets the corresponding bit in this array to true.
	 *
	 * - This process is also valid for unmapped node IDs detected in MSG1/2 callbacks.
	 *
	 * This prevents repeated warnings for the same unknown node ID, reducing log spam.
	 */
	bool 		_unknown_addr_logged[128]{};

	bool 		banned_node_ids[128];						// TODO: Parse them from param once in init and param update and treat any hit as hard fail (don’t flip _can_control_ready true).

	ESCStatus _esc_data[MAX_ACTUATORS] {};

	param_t _param_handles[MAX_ACTUATORS] {PARAM_INVALID};

	/* UAVCANv0 method binder aliases */
       	typedef uavcan::MethodBinder<UavcanHwescController *,
       	       	void (UavcanHwescController::*)(const uavcan::TimerEvent &) >
	       	TimerCbBinder;
	typedef uavcan::MethodBinder < UavcanHwescController *,
             	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &) >
             	GetEscIDCbBinder;

	typedef uavcan::MethodBinder < UavcanHwescController *,
               	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &) >
               	StatusMsg1CbBinder;

       	typedef uavcan::MethodBinder < UavcanHwescController *,
               	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &) >
               	StatusMsg2CbBinder;

       	// typedef uavcan::MethodBinder < UavcanHwescController *,
        //       	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &) >
        //       	StatusMsg3CbBinder;

       	//typedef uavcan::MethodBinder < UavcanHwescController *,
        // 	void (UavcanHwescController::*)(const uavcan::ServiceCallResult<com::hobbywing::esc::GetEscID> &)>
	//	GetEscIDCbBinder;

	/* UAVCAN Related Members */
	uavcan::TimerEventForwarder<TimerCbBinder> 				_timer;			///< periodic timer

	uavcan::MonotonicTime							_prev_cmd_pub;   	///< rate limiting
	uavcan::INode								&_node;			///< reference to UAVCAN node

	uavcan::Publisher<com::hobbywing::esc::GetEscID>			_pub_get_esc_id;	///< GetEscID service client
	uavcan::Subscriber<com::hobbywing::esc::GetEscID, GetEscIDCbBinder> 	_sub_get_esc_id;	///< GetEscID service response subscriber

	uavcan::Publisher<com::hobbywing::esc::RawCommand>			_uavcan_pub_raw_cmd;	///< RawCommand publisher

       	uavcan::Subscriber<com::hobbywing::esc::StatusMsg1, StatusMsg1CbBinder> _sub_status_msg1;	///< StatusMsg1 subscriber
       	uavcan::Subscriber<com::hobbywing::esc::StatusMsg2, StatusMsg2CbBinder> _sub_status_msg2;	///< StatusMsg2 subscriber
       	// uavcan::Subscriber<com::hobbywing::esc::StatusMsg3, StatusMsg3CbBinder> _sub_status_msg3;	///< StatusMsg3 subscriber

	/* uORB related members */
	esc_status_s	_esc_status{};

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	uORB::Subscription							_actuator_armed_sub{ORB_ID(actuator_armed)};

	/* ------------------------- UAVCAN Functions ------------------------- */

	/**
	 * Callback for StatusMsg1 messages from ESCs.
	 * The callback runs several checks before accepting the message:
	 * 1. If the driver is not in the mapping process.
	 * 2. Validates if the node ID is not banned.
	 * 3. Finds the corresponding ESC index for the node ID and validates it.
	 * Afterwards, the message is processed and the relevant data is extracted to the ESCStatus structure.
	 * Timestamps and flags are updated accordingly.
	 *
	 * @param msg The received StatusMsg1 message
	 */
	void status_msg1_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg);

	/**
	 * Callback for StatusMsg2 messages from ESCs.
	 * The callback performs several checks before processing the message:
	 * 1. If the driver is not in the mapping process.
	 * 2. Validates if the node ID is not banned.
	 * 3. Finds the corresponding ESC index for the node ID and validates it.
	 * Afterwards, the message is processed and the relevant data is extracted to the ESCStatus structure.
	 * Timestamps and flags are updated accordingly.
	 *
	 * @param msg The received StatusMsg2 message
	 */
	void status_msg2_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg);

	// void status_msg3_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg);

	/**
	 * Send GetEscID UAVCAN broadcast to all nodes.
	 *
	 * @return 0 on success, negative error code on failure
	 */
	int get_esc_id_req();

	/**
	 * Callback for GetEscID service responses from ESCs.
	 * The callback processes the response to extract the node ID and throttle channel.
	 * It performs validation checks on the received data and updates the ESCStatus structure accordingly which are:
	 * 1. If the received node ID is a valid one.
	 * 2. If the received throttle channel corresponds to a valid ESC index.
	 * 3. If the node ID is not banned.
	 * 4. Duplicate node ID and index checks.
	 * 5. If the ESC is expected based on the configured rotor count.
	 * If any validation fails, appropriate error bits are set, and a remap is requested.
	 *
	 * ESCStatus structure variables, timestamps and flags are updated upon reception of the GetEscID response.
	 *
	 * @param msg The received GetEscID response message
	 */
	void get_esc_id_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &result);

	/* ------------------------- Driver Operation Functions ------------------------- */

	/**
	 * Timer callback for periodic updates.
	 *
	 * @param event The timer event triggering the update.
	 */
	void periodic_update(const uavcan::TimerEvent &);

	/**
	 * Check if the ESCs are ready to publish uORB esc_status by MSG1 freshness of all online ESCs.
	 *
	 * Note: Preliminarily ensure that the system is not in the mapping process.
	 *
	 * @return true if ready to publish, false otherwise.
	 */
	bool is_ready_to_publish() const;

	/**
	 * Publish esc_status uORB topic.
	 *
	 * Collect MSG1 and MSG2 data from ESCs and publish esc_status uORB topic altogether.
	 *
	 * This topic contains information from MSG1 and MSG2.
	 */
	void publish_esc_status();

	/**
	 * Check for timeouts of ESC messages.
	 *
	 * This function checks if any ESC has timed out based on the last received message timestamps.
	 * It only updates the active status of each ESC.
	 */
	void check_timeouts();

	/**
	 * Evaluate all dark ESCs and attempt remapping if necessary.
	 *
	 * This function checks the status of all connected ESCs and attempts to remap any that are
	 * detected as dark (not responding). It is typically called periodically to ensure all ESCs
	 * are functioning correctly.
	 */
	void evaluate_all_dark(hrt_abstime now);

	/* ------------------------- Driver State Management Functions ------------------------- */

	/**
	 * Handle arm state changes.
	 *
	 * @param armed The new arm state.
	 */
	void handle_arm_state_change(bool armed);

	/**
	 * Start Mapping Process
	 *
	 * If the system is not armed, not in a mapping process already and does need a mapping:
	 * This function will initiate the mapping process.
	 *
	 * Reset previous mapping data
	 *
	 * Set configured ESCs on ESCStatus can_throttle flag according to _param_handles[]
	 *
	 * Call GetEscID service client
	 */
	void start_mapping();

	/**
	 * Finalize Mapping Process
	 *
	 * This function finalizes the mapping process after collecting responses from ESCs.
	 * It validates the collected data that establish a mapping between ESC indices and node IDs,
	 * and perform a state transition. It sets the operational state based on the validity of the mapping:
	 * - If the mapping is valid, it marks the system as ready for CAN control and telemetry.
	 * - If the mapping is invalid, it sets appropriate error bits and marks the system as not ready for CAN control and/or telemetry.
	 *
	 */
	void finalize_mapping();

	/**
	 * Query a remap of ESCs.
	 *
	 * This function sets the _need_remap flag to true, indicating that a remap of ESCs is required.
	 */
	void request_remap();

	/**
	 * Apply backoff strategy for mapping failures.
	 *
	 * This function increases the remap backoff duration exponentially up to a maximum limit.
	 * It is called after a mapping failure to prevent immediate subsequent remap attempts.
	 */
	void apply_mapping_failure_backoff();

	/**
	 * Reset the remap backoff to its initial value.
	 *
	 * This function resets the remap backoff duration to the base value and clears the remap attempt counter.
	 * It is typically called after a successful mapping to prepare for future remap attempts.
	 */
	void reset_remap_backoff();

	/**
	 * Attempt to remap ESCs if conditions are met.
	 *
	 * This function checks if a remap is needed and if the system is not currently armed or in a mapping process.
	 * If conditions are met, it initiates the remapping process.
	 *
	 * @param now The current time in microseconds.
	 */
	void attempt_remap(hrt_abstime now);

	/**
	 * Update the readiness flags based on the current state of the ESCs.
	 */
	void update_readiness_flags();

	/* ------------------------- Driver Error Management Functions ------------------------- */

	/**
	 * Set given error bit in the driver state.
	 *
	 * @param bit The error bit to set.
	 * @param ctrl If true, set the error bit for control.
	 * @param telem If true, set the error bit for telemetry.
	 */
	void set_error(HwescErrorBit bit, bool ctrl, bool telem);

	/**
	 * Clear given error bits in the driver state.
	 *
	 * @param mask The bitmask of error bits to clear.
	 * @param ctrl If true, clear the error bits for control.
	 * @param telem If true, clear the error bits for telemetry.
	 */
	void clear_error_mask(uint32_t mask, bool ctrl, bool telem);

	/**
	 * Clear all mapping related error bits in the driver state.
	 *
	 * Calls clear_error_mask() to clear all mapping related error bits.
	 */
	void clear_mapping_error_bits();

	/**
	 * Check if the given error bit is related to mapping errors.
	 *
	 * @param bit The error bit to check.
	 * @return true if the error bit is related to mapping errors, false otherwise.
	 */
	bool is_mapping_error(HwescErrorBit bit) const;

	/* ------------------------- Helper Functions - GENERIC ------------------------- */

	/** Convert throttle channel to ESC index */
	static inline uint8_t 	throttle_ch2esc_index(uint8_t ch) {
		return (ch >= 1 && ch <= esc_status_s::CONNECTED_ESC_MAX) ? uint8_t(ch - 1) : kInvalidIdx;
	}
	/** Convert ESC index to throttle channel */
	static inline uint8_t 	esc_index2throttle_ch(uint8_t idx) {
		return (idx < esc_status_s::CONNECTED_ESC_MAX) ? uint8_t(idx + 1) : 0;
	}

	/**
	 * Check if a given node ID is in the banned list.
	 *
	 * @param node_id The node ID to check.
	 * @return true if the node ID is banned, false otherwise.
	 */
	bool is_banned_node(uint8_t node_id) const;

	/* ------------------------- Helper Functions - ESCStatus Struct ------------------------- */

	/**
	 * Fetch the ESC index for a given node ID from the current mapping.
	 *
	 * @param node_id The node ID of the ESC.
	 * @return The corresponding ESC index if found, kInvalidIdx otherwise.
	 */
	uint8_t fetch_node_esc_index(uint8_t node_id) const;

	/**
	 * Check if the ESC at the given index is configured for CAN throttle control via parameter UAVCAN_EC_FUNCx.
	 *
	 * @param esc_index The index of the ESC to check.
	 * @return true if the ESC is configured for CAN throttle control, false otherwise.
	 */
	bool is_can_throttle(uint8_t esc_index) const;

	/**
	 * Find duplicate node ID in the current mapping excluding a specific ESC index.
	 *
	 * @param nid The node ID to check for duplicates.
	 * @param exclude_idx The ESC index to exclude from the check.
	 * @return The index of the duplicate ESC if found, kInvalidIdx otherwise.
	 */
	int find_duplicate_node_id(uint8_t nid, uint8_t exclude_idx) const;
};
