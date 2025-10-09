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

	int init();

	void update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned total_outputs);

	esc_status_s &esc_status() { return _esc_status; }

	/**
	 * Sets the number of rotors and enable timer
	 */
	void set_rotor_count(uint8_t count);

	static constexpr int INT14_MAX = 8191;
	static constexpr int INT14_MIN = -8192; // not used for HWESC
	static int max_output_value() { return INT14_MAX; }

private:
	static constexpr uint8_t kInvalidIdx = 0xFF;   // invalid esc index
	static constexpr uint8_t kInvalidNID = 0x00;   // invalid node id
	struct ESCStatus {
		bool     is_expected{false};	// expected ESCs according to _rotor_count regardless of physical actuator link
		bool     is_hobbywing{false};	// might need it later, will assign it regardless
		bool     can_throttle{false};	// whether this esc index is configured to be controlled via CAN
		bool     duplicate_mapping{false};

		uint8_t  node_id{kInvalidNID};
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
		hrt_abstime last_seen_us{0};

		bool 	 msg1_received{false};
		bool 	 msg2_received{false};
		// bool  msg3_received{false};
		bool	 active{false};
	};

	bool _reinit_requested{true}; 		// < trigger for mapping the escs.
	bool _mapping_in_progress{false};

	bool _armed{false};			// < whether the system is armed or not

	bool _can_control_ready{false};		// < whether the system is ready for CAN control (set to true if PWM regardless for now)

	bool _telem_ok{false};			// < whether telemetry is OK to publish

	// Enum for error types
 	enum EscErrorType {
		HWESC_OK = 0,
		HWESC_DUPLICATE_INDEX,		// multiple escs responded with same throttle channel in GetEscID
		HWESC_INVALID_INDEX,		// invalid esc index in GetEscID response
		HWESC_UNEXPECTED_INDEX,		// ch > rotor_count when rotor_count is used as hard expectation
		HWESC_DUPLICATE_ADDR,		// multiple escs responded with same node id
		HWESC_INVALID_ADDR,		// banned node id
		HWESC_UNDISC_ADDR,		// runtime message from an unmapped node/esc
		HWESC_MISSING_ESC,		// expected index not seen in mapping window
		HWESC_MISSING_ESC_WARN,		// expected index not seen in mapping window - warn only
		HWESC_MAP_FAILED,		// GetEscID call error
		HWESC_MAP_TIMEOUT,		// no mapping responses within window
		HWESC_MSG_TIMEOUT,		// runtime MSG1/2 timeout after previously mapped
		// TODO: add errors related to expected escs not initialized/mapped
	};

	// uint8_t _esc_index_by_node_id[128]; 					// node_id (1..0x7D) -> esc_index (0..7) map
	// uint8_t _node_id_by_index[esc_status_s::CONNECTED_ESC_MAX];		// esc_index -> node_id map

	uint16_t _status_counter{0};						// < internal counter for esc status topic

	static constexpr uint64_t BOOT_GRACE_US            = 2'000'000;  		// after boot/mapping, don't mark inactive too early
	static constexpr uint64_t TIMEOUT_MSG1_US          = 700'000;  			// heartbeat (MSG1) timeout
	static constexpr uint64_t TIMEOUT_MSG2_US          = 700'000;  			// slower, informational
	static constexpr uint64_t CHECK_TIMEOUTS_MIN_US    = 100'000;    		// don't scan too often (10 Hz)
	static constexpr uint64_t GET_ESCID_COOLDOWN_US    = 1'000'000 / MAX_GET_RATE_HZ;
	static constexpr uint64_t MAPPING_COLLECTION_WINDOW_US = 500'000; 		// 500 ms window to gather GetEscID responses
	static constexpr uint64_t WARN_THROTTLE_INTERVAL_US    = 1'000'000; 		// 1 second


	hrt_abstime _last_esc_id_query{0};						// < last time we queried for GetEscID
	hrt_abstime _boot_time_us{hrt_absolute_time()};					// < system time since boot
	hrt_abstime _last_timeout_check{0};						// < last time we checked for timeouts
	hrt_abstime _mapping_deadline_us{0};
	hrt_abstime _last_invalid_index_warn{0};

	ESCStatus _esc_data[MAX_ACTUATORS] {};

	// message callbacks

	void status_msg1_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg);
	void status_msg2_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg);
	// void status_msg3_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg);

	int get_esc_id_req();
	void get_esc_id_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &result);

	void map_and_assign_all_once();
	void finalize_mapping_window();
	int error_handler(uint8_t error_type, uint8_t val, int res);

	bool is_can_throttle(uint8_t esc_index) const;

	bool has_duplicate_node_id(uint8_t nid, uint8_t exclude_idx) const;

	void periodic_update(const uavcan::TimerEvent &);

	/**
	 * Publish esc_status uORB topic if ESCs are ready.
	 *
	 * This topic contains information from MSG1 and MSG2.
	 */
	void maybe_publish_esc_status();

	/**
	 * Publish esc_hw_status uORB topic if ESCs are ready.
	 *
	 * This topic contains information from MSG3, and other data
	 * not fit to esc_status such as PWM, status flags, temperatures.
	 *
	*/
	// void maybe_publish_esc_hw_status();

	void check_timeouts();

	static inline uint8_t 	throttle_ch2esc_index(uint8_t ch) {
		return (ch >= 1 && ch <= esc_status_s::CONNECTED_ESC_MAX) ? uint8_t(ch - 1) : kInvalidIdx;
	}
	static inline uint8_t 	esc_index2throttle_ch(uint8_t idx) {
		return (idx < esc_status_s::CONNECTED_ESC_MAX) ? uint8_t(idx + 1) : 0;
	}

	uint8_t fetch_node_esc_index(uint8_t node_id) const;

	bool banned_node_ids[128];	// TODO: Parse them from param once in init and param update and treat any hit as hard fail (don’t flip _can_control_ready true).
	bool is_banned_node(uint8_t node_id) const;

	// UAVCANv0 method binder aliases
	typedef uavcan::MethodBinder < UavcanHwescController *,
               	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &) >
               	StatusMsg1CbBinder;


       	typedef uavcan::MethodBinder < UavcanHwescController *,
               	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &) >
               	StatusMsg2CbBinder;


       	// typedef uavcan::MethodBinder < UavcanHwescController *,
        //       	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &) >
        //       	StatusMsg3CbBinder;

	typedef uavcan::MethodBinder < UavcanHwescController *,
             	void (UavcanHwescController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &) >
             	GetEscIDCbBinder;


       	//typedef uavcan::MethodBinder < UavcanHwescController *,
        // 	void (UavcanHwescController::*)(const uavcan::ServiceCallResult<com::hobbywing::esc::GetEscID> &)>
	//	GetEscIDCbBinder;


       	typedef uavcan::MethodBinder<UavcanHwescController *,
       	       	void (UavcanHwescController::*)(const uavcan::TimerEvent &)>
	       	TimerCbBinder;

	uint8_t		_rotor_count{0};
	uint8_t		_uavcan_rotor_count{0};

	esc_status_s	_esc_status{};

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	uORB::Subscription							_actuator_armed_sub{ORB_ID(actuator_armed)};

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   	///< rate limiting
	uavcan::INode								&_node;

	uavcan::Publisher<com::hobbywing::esc::RawCommand>			_uavcan_pub_raw_cmd;

       	uavcan::Subscriber<com::hobbywing::esc::StatusMsg1, StatusMsg1CbBinder> _sub_status_msg1;
       	uavcan::Subscriber<com::hobbywing::esc::StatusMsg2, StatusMsg2CbBinder> _sub_status_msg2;
       	// uavcan::Subscriber<com::hobbywing::esc::StatusMsg3, StatusMsg3CbBinder> _sub_status_msg3;

	uavcan::Publisher<com::hobbywing::esc::GetEscID>			_pub_get_esc_id;
	uavcan::Subscriber<com::hobbywing::esc::GetEscID, GetEscIDCbBinder> 	_sub_get_esc_id;

	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	param_t _param_handles[MAX_ACTUATORS] {PARAM_INVALID};
};
