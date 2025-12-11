#pragma once

#include <uORB/Publication.hpp>
// #include <uORB/topics/esc_hw_status.h>
#include <uORB/topics/esc_status.h>

#include <uavcan/uavcan.hpp>
#include <com/hobbywing/esc/GetEscID.hpp>
#include <com/hobbywing/esc/StatusMsg1.hpp>
#include <com/hobbywing/esc/StatusMsg2.hpp>
// #include <com/hobbywing/esc/StatusMsg3.hpp>

#include <px4_platform_common/module_params.h>

/** Hobbywing-ESC driver constants
 *
 *  Keep *all* protocol- and timing-specific values here so they
 *  have a single point of truth leak into the .cpp. Switch to
 *  PX4 parameters if run-time tuning is needed.
 */
namespace hwesc
{
/* ---------- protocol limits ------------------------------------------------ */
static constexpr int  MAX_ESC             = 8;     		///< max motors we support
static constexpr uint8_t INVALID_TID      = 0xFF;  		///< sentinel for “unmapped”

/* ---------- UAVCAN details -------------------------------------------------- */
static constexpr int  NODE_ID_COUNT       = 127;   		///< 0–127 protocol-specific

/* ---------- timing (µs unless noted) --------------------------------------- */
static constexpr unsigned TIMER_HZ        = 10;           	///< driver work-loop rate
static constexpr uint64_t STATUS_TIMEOUT  = 2'000'000ULL; 	///< 2 s
static constexpr uint64_t DISCOVERY_TIMEOUT = 3'000'000ULL;	///< 3 s
static constexpr uint64_t GET_ID_FAST_PERIOD = 100'000ULL;	///< 0.1 s
static constexpr uint64_t GET_ID_SLOW_PERIOD = 1'000'000ULL;	///< 1 s

/* ---------- scaling factors ------------------------------------------------- */
static constexpr float DECI2V             = 0.1f;  		///< deciVolt → Volt
static constexpr float DECI2A             = 0.1f;  		///< deciAmp  → Amp
} // namespace hwesc

enum class IdScanMode { FAST, SLOW };

struct ESCStatus {
	uint8_t throttle_id;
	uint8_t node_id;
	bool	online{false};
	uint16_t rpm{0};
	uint16_t pwm{0};
	uint16_t status_flags;
	uint16_t voltage_mv{0};
	uint16_t current_ma{0};
	uint8_t temperature_deg{0};
	// uint8_t mos_temp_deg{0};
	// uint8_t cap_temp_deg{0};
	// uint8_t motor_temp_deg{0};

	uint64_t timestamp_msg1{};
	uint64_t timestamp_msg2{};
	// uint64_t timestamp_msg3{};
	uint64_t timestamp_id{};
	bool msg1_received{false};
	bool msg2_received{false};
	// bool msg3_received{false};
};

class UavcanHWingEscDriver : public ModuleParams
{
public:
	UavcanHWingEscDriver(uavcan::INode &node);
	~UavcanHWingEscDriver() = default;

	// setup periodic updater
	int init();

private:


	// Status info for each throttle_id (index 0..MAX_ESC-1)
	ESCStatus _esc_data[hwesc::MAX_ESC] {};

	int find_index_by_node(uint8_t node_id) const;
	void maybe_publish_status(uint64_t now);
	void discover_nodes(uint64_t now);
	bool check_online(int idx, uint64_t now);

	void hwesc_statusmsg1_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg);
	void hwesc_statusmsg2_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg);
	// void hwesc_statusmsg3_sub_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg);

	void get_esc_id_rx_cb(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID>&);

	IdScanMode _scan_mode{IdScanMode::FAST};

	hrt_abstime _last_esc_id_query{0};
	hrt_abstime _discovery_start{0};

	typedef uavcan::MethodBinder < UavcanHWingEscDriver *,
		void (UavcanHWingEscDriver::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &) >
		StatusMsg1CbBinder;

	typedef uavcan::MethodBinder < UavcanHWingEscDriver *,
		void (UavcanHWingEscDriver::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &) >
		StatusMsg2CbBinder;

	//typedef uavcan::MethodBinder < UavcanHWingEscDriver *,
	//	void (UavcanHWingEscDriver::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &) >
	//	StatusMsg3CbBinder;

	typedef uavcan::MethodBinder < UavcanHWingEscDriver *,
		void (UavcanHWingEscDriver::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &) >
		GetEscIDCbBinder;

	//typedef uavcan::MethodBinder < UavcanHWingEscDriver *,
	//	void (UavcanHWingEscDriver::*)(const uavcan::ServiceCallResult<com::hobbywing::esc::GetEscID> &)> GetEscIDCallback;

	typedef uavcan::MethodBinder<UavcanHWingEscDriver *,
		void (UavcanHWingEscDriver::*)(const uavcan::TimerEvent &)>
		TimerCbBinder;

	void periodic_update(const uavcan::TimerEvent &);

	//void get_esc_id_cb(const uavcan::ServiceCallResult<com::hobbywing::esc::GetEscID> &result);

	//uavcan::ServiceClient<com::hobbywing::esc::GetEscID, GetEscIDCallback> _get_esc_id_client;
	uavcan::INode &_node; // In initializer, INode comes first.

	uavcan::Publisher<com::hobbywing::esc::GetEscID> _get_esc_id_pub;

	uavcan::Subscriber<com::hobbywing::esc::StatusMsg1, StatusMsg1CbBinder> _sub_status_msg1;
	uavcan::Subscriber<com::hobbywing::esc::StatusMsg2, StatusMsg2CbBinder> _sub_status_msg2;
	// uavcan::Subscriber<com::hobbywing::esc::StatusMsg3, StatusMsg3CbBinder> _sub_status_msg3;
	uavcan::Subscriber<com::hobbywing::esc::GetEscID, GetEscIDCbBinder> _sub_get_esc_id;

	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	// uORB::Publication<esc_hw_status_s> _hwesc_status_pub{ORB_ID(esc_hw_status)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
};
