#include "hwesc.hpp"

UavcanHWingEscDriver::UavcanHWingEscDriver(uavcan::INode &node) :
	ModuleParams(nullptr),
	_node(node),	// _node must appear before anything that takes it by reference
	_get_esc_id_pub(node),
	_sub_status_msg1(node),
	_sub_status_msg2(node),
	// _sub_status_msg3(node),
	_sub_get_esc_id(node),
	_timer(node)
{

}
int UavcanHWingEscDriver::init() {
	int res = _get_esc_id_pub.init();
	if (res < 0) return res;

	res = _sub_status_msg1.start(StatusMsg1CbBinder(this, &UavcanHWingEscDriver::hwesc_statusmsg1_sub_cb));
	if (res < 0) return res;

	res = _sub_status_msg2.start(StatusMsg2CbBinder(this, &UavcanHWingEscDriver::hwesc_statusmsg2_sub_cb));
	if (res < 0) return res;

	// res = _sub_status_msg3.start(StatusMsg3CbBinder(this, &UavcanHWingEscDriver::hwesc_statusmsg3_sub_cb));
	// if (res < 0) return res;

	res = _sub_get_esc_id.start(GetEscIDCbBinder(this, &UavcanHWingEscDriver::get_esc_id_rx_cb));
	if (res < 0) return res;

	_timer.setCallback(TimerCbBinder(this, &UavcanHWingEscDriver::periodic_update));
	_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / hwesc::TIMER_HZ));

	_discovery_start = hrt_absolute_time();

	return 0;
}

void UavcanHWingEscDriver::periodic_update(const uavcan::TimerEvent &) {
	const uint64_t now = hrt_absolute_time();

	uint64_t period = (_scan_mode == IdScanMode::FAST) ? hwesc::GET_ID_FAST_PERIOD : hwesc::GET_ID_SLOW_PERIOD;

	if (hrt_elapsed_time(&_last_esc_id_query) > period) {
		com::hobbywing::esc::GetEscID msg;
		msg.payload.resize(1);
		msg.payload[0] = 0x00;                    // command byte
		_get_esc_id_pub.broadcast(msg);
		_last_esc_id_query = now;
	}

	maybe_publish_status(now);

	// Duplicate PWM values and send over CAN if CAN Throttle configured
	// MAP PWM to CAN Throttle range
}

void UavcanHWingEscDriver::get_esc_id_rx_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID>& msg)
{
	_scan_mode = IdScanMode::SLOW;

	if (msg.payload[0] == 0) { return; }          // ignore broadcasts, update later on

	const uint8_t node_id      = msg.payload[0];
	const uint8_t throttle_id  = msg.payload[1];	// What we get here can range from 1 to 8. Doesn't start from 0.

	if (throttle_id > hwesc::MAX_ESC) { PX4_WARN("Bad throttle_id %u from node %u", throttle_id, node_id); return; }

	if (throttle_id < 1) { PX4_WARN("Bad throttle_id %u from node %u", throttle_id, node_id); return; }

	int idx = throttle_id-1;

	ESCStatus &esc = _esc_data[idx];

	if (esc.node_id && esc.node_id != node_id) {
		PX4_ERR("Throttle %u claimed by node %u and %u", throttle_id, esc.node_id, node_id);
		return;
	}

	esc.online	 = true;

	esc.throttle_id  = throttle_id;
	esc.node_id      = node_id;
	esc.timestamp_id = hrt_absolute_time();
}

int UavcanHWingEscDriver::find_index_by_node(uint8_t node_id) const
{
    for (int i = 0; i < hwesc::MAX_ESC; ++i) {
        if (_esc_data[i].node_id == node_id) {
            return i;
        }
    }
    return -1;                  // unmapped
}


void UavcanHWingEscDriver::hwesc_statusmsg1_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg) {
	int idx = find_index_by_node(msg.getSrcNodeID().get());
	if (idx < 0) {
		PX4_WARN("Invalid ESC in StatusMsg1");
		return;
	}
	auto &esc = _esc_data[idx];

	esc.rpm = msg.rpm;
	esc.pwm = msg.pwm;
	esc.status_flags = msg.status;

	esc.msg1_received = true;
	esc.timestamp_msg1 = hrt_absolute_time();
}

void UavcanHWingEscDriver::hwesc_statusmsg2_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg) {
	int idx = find_index_by_node(msg.getSrcNodeID().get());
	if (idx < 0) {
		PX4_WARN("Invalid ESC in StatusMsg2");
		return;
	}
	auto &esc = _esc_data[idx];

	// Raw values need to be divided by 10 when publishing
	esc.voltage_dv = msg.input_voltage;  	// in deciVolts (V * 10)
	esc.current_da = msg.current;  		// in deciAmps (A * 10)
	esc.temperature_deg = msg.temperature;

	esc.msg2_received = true;
	esc.timestamp_msg2 = hrt_absolute_time();
}

// void UavcanHWingEscDriver::hwesc_statusmsg3_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg) {
//	int idx = find_index_by_node(msg.getSrcNodeID().get());
//	if (idx < 0) {
//		PX4_WARN("Invalid ESC in StatusMsg3");
//		return;
//	}
//	auto &esc = _esc_data[idx];
//
//	esc.mos_temp_deg = msg.MOS_T;
//	esc.cap_temp_deg = msg.CAP_T;
//	esc.motor_temp_deg = msg.Motor_T;
//
//	esc.msg3_received = true;
//	esc.timestamp_msg3 = hrt_absolute_time();
//}

bool UavcanHWingEscDriver::check_online(int idx, uint64_t now)
{
	ESCStatus &esc = _esc_data[idx];

	if (esc.online && (hrt_elapsed_time(&esc.timestamp_id) > hwesc::STATUS_TIMEOUT)) {
		// Status Message Timeout!
		esc.online = false;
		// Transition logging
		PX4_WARN("ESC %d (node %d) heartbeat timed out", esc.throttle_id, esc.node_id);
		// Activate discovery
		discover_nodes(now);
		// Return early
		return esc.online;
	}

	bool msg1_ok = esc.msg1_received && (hrt_elapsed_time(&esc.timestamp_msg1) < hwesc::STATUS_TIMEOUT);
	bool msg2_ok = esc.msg2_received && (hrt_elapsed_time(&esc.timestamp_msg2) < hwesc::STATUS_TIMEOUT);
	// bool msg3_ok = esc.msg3_received && (hrt_elapsed_time(&esc.timestamp_msg3) < hwesc::STATUS_TIMEOUT);

	bool status_ok = msg1_ok && msg2_ok; // && msg3_ok;

	if (esc.online && !status_ok) {
		// ESC is discovered but messages are not being received.
		esc.online = false;
		// Transition logging
		PX4_WARN("ESC %d (node %d) status messages timed out", esc.throttle_id, esc.node_id);
		// Activate discovery
		discover_nodes(now);
		// Return early
		return esc.online;
	}

	if (!esc.online) {
		return esc.online;
	}

	return esc.online;
}

void UavcanHWingEscDriver::discover_nodes(uint64_t now)
{
	// Check if it is slow, otherwise discovery never timeouts.
	if (_scan_mode == IdScanMode::SLOW) {
		_scan_mode = IdScanMode::FAST;
		_discovery_start = now;
	}
	else if (hrt_elapsed_time(&_discovery_start) > hwesc::DISCOVERY_TIMEOUT) {
		// Already been discovering and no success, return to slow mode
		_scan_mode = IdScanMode::SLOW;
	}
}

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

void UavcanHWingEscDriver::maybe_publish_status(uint64_t now)
{
	// Publish MSG1 and MSG2 data to esc_status uORB topic whenever possible.
	esc_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	status.esc_online_flags = 0;

	uint8_t online_count = 0;

	for (int i = 0; i <hwesc::MAX_ESC; ++i) {
		const ESCStatus &e = _esc_data[i];

		bool esc_online = check_online(i,now);

		if (esc_online) {
			++online_count;

			status.esc[i].timestamp       	= status.timestamp;
			status.esc[i].esc_address     	= e.node_id;
			status.esc[i].esc_voltage     	= e.voltage_dv * 0.1f;
			status.esc[i].esc_current     	= e.current_da * 0.1f;
			status.esc[i].esc_temperature 	= e.temperature_deg;
			status.esc[i].esc_rpm         	= e.rpm;
			status.esc[i].esc_power 	= static_cast<int8_t>((e.pwm / 8191.0f) * 100.0f);
			status.esc[i].failures	    	= statusflags_to_failures(e.status_flags);
			status.esc[i].esc_errorcount  	= 0;		// TODO: add error count if available

			status.esc_online_flags |= (1u << i);

			// Clear edge flags for next cycle so don't report same data again.
			_esc_data[i].msg1_received = false;
			_esc_data[i].msg2_received = false;
		} else {
			// Not online.
		}

	}
	status.esc_count = online_count;
	status.counter++;

	_esc_status_pub.publish(status);
}

