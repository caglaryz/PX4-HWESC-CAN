/****************************************************************************
 *
 * Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file authenticator.cpp
 *
 * UAVCAN node authentication helper using com.h3robotics.GetSHA1Key.
 *
 * @author Caglar Yilmaz <yilmaz.caglar@tubitak.gov.tr>
 */

#include "authenticator.hpp"

#include <string.h>
#include <inttypes.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

UavcanAuthenticator::UavcanAuthenticator(uavcan::INode &node) :
	ModuleParams(nullptr),
	_node(node),
	_getsha1_client(node)
{
	/* The constructor performs no work – call init() so that failures can be
	 * propagated back to the caller when bringing up the driver. */
}

const uint8_t UavcanAuthenticator::kAuthSeed[SHA1_LEN] = {
    0x94, 0xe3, 0x17, 0xad, 0x64, 0x3d, 0xba, 0x22, 0x09, 0x35,
    0xdf, 0x3c, 0x72, 0x4a, 0xc3, 0x35, 0x88, 0x9e, 0x61, 0xf2
}; // SHA1("cheesecake")

int
UavcanAuthenticator::init()
{
	/* Read configuration parameters and prepare the shared service client. */
	(void)param_get(param_find("UAVCAN_AUTH_MODE"), &_mode);

	int32_t mask = 0;
	(void)param_get(param_find("UAVCAN_AUTH_MASK"), &mask);
	_mask = static_cast<uint32_t>(mask);

	_active = (_mode > 0);

	if (_mode > 0) {
		/* Prepare the request payload with the shared secret (V0.1 - will be modified to dynamic hashing) */
		memcpy(_request_payload, kAuthSeed, SHA1_LEN);
		_getsha1_client.setCallback(GetSHA1Callback(this, &UavcanAuthenticator::cb_getsha1key));
	}

	return PX4_OK;
}

void
UavcanAuthenticator::update()
{
	/* Rate limit the updates*/
	hrt_abstime now = hrt_absolute_time();
	if (now - _last_update_time < kPeriod) {
		return;
	}
	_last_update_time = now;

	/* Inform external modules via updating custom uORB uavcan_authentication_check.*/
	bool system_ok = auth_system_ok();
	_uavcan_auth_status.timestamp = now;
	_uavcan_auth_status.system_authenticated = system_ok;
	_uavcan_auth_status.failed_devices_mask = get_failed_devices_mask(system_ok);
	_auth_status_pub.publish(_uavcan_auth_status);

	/* Check arming status of the drone via uORB subscription
	 * If the system is armed, never process authentication requests.*/
	set_armed(); 		// update _armed state again!
	if (!auth_ready()) return;

	/* Iterate over all tracked nodes and issue requests as needed. */
	for (uint8_t i = 0; i < kMaxEntries; i++) {
		auto &e = _tab[i];
		if (e.node_id == kInvalidNode) 			continue;	// empty slot
		if (did_timeout(e.last_seen_time)) {
			remove_entry(i);					// node offline timeout
			continue;
		}
		if (e.state == State::OK) 			continue;	// already authed
		if (e.attempts >= kMaxRetries) 			continue;	// too many attempts to issue a new query
		if (now - e.last_request_time < kDelay) 	continue;	// wait for cooldown
		/* Issue authentication request */
		try_auth_idx(i);
	}
}

bool
UavcanAuthenticator::auth_ready() const
{
	if (_active && !is_armed()) return true;
	else return false;
}

uint32_t
UavcanAuthenticator::get_failed_devices_mask(bool system_ok) const
{
	if (system_ok) {
		return 0;
	}

	uint32_t mask = 0;
	for (const auto &e : _tab) {
		if (e.state == State::FAIL && e.type_bit >= 0 && e.type_bit < 32) {
			mask |= (1u << e.type_bit);
		}
	}
	return mask;
}

bool
UavcanAuthenticator::is_armed() const
{
	return _armed;
}

void
UavcanAuthenticator::set_armed()
{
	/* Update the arming state from uORB topic subscription.*/
	if (_actuator_armed_sub.updated()) {
		actuator_armed_s actuator_armed;
		if (_actuator_armed_sub.copy(&actuator_armed)) {
			if (actuator_armed.armed != _armed) {
				_armed = actuator_armed.armed;
			}
		}
	}
}

int
UavcanAuthenticator::name_to_bit(const char* n)
{
	struct Pair { const char* name; int bit; };
	static constexpr Pair map[] = {
		{"accel",0},{"airspeed",1},{"baro",2},{"battery",3},
		{"differential_pressure",4},{"flow",5},{"fuel_tank_status",6},
		{"gnss",7},{"gnss_relative",8},{"gyro",9},
		{"hygrometer_sensor",10},{"ice_status",11},{"mag",12},
		{"rangefinder",13},{"safety_button",14},
	};
	for (unsigned i=0;i<sizeof(map)/sizeof(map[0]);i++) {
		if (strcmp(n, map[i].name)==0) return map[i].bit;
	}
	return -1;
}

bool
UavcanAuthenticator::is_auth_required(const char* bridge_name) const
{
	const int bit = name_to_bit(bridge_name);
	if (bit < 0) return false;
	return (_mask & (1u << bit)) != 0u;
}

int
UavcanAuthenticator::find_slot_by_nid(uint8_t nid) const
{
	for (int i=0;i<kMaxEntries;i++) if (_tab[i].node_id == nid) return i;
	return -1;
}
int
UavcanAuthenticator::find_free_slot() const
{
	for (int i=0;i<kMaxEntries;i++) if (_tab[i].node_id == kInvalidNode) return i;
	return -1;
}
int
UavcanAuthenticator::find_reclaimable_slot() const
{
	/* TODO: Redesign table pressure mitigation.
	 * 1 - scan for the stalest NEW (largest now - last_seen_time)
	 * 2 - else scan for stalest OK,
	 * 3 - then fallback to 0. */

	// Prefer reclaiming FAIL entries; keep OK and NEW
	for (int i=0;i<kMaxEntries;i++) if (_tab[i].state == State::FAIL) return i;
	return 0; // as last resort, take slot 0
}

void
UavcanAuthenticator::reset_()
{
	for (auto &e : _tab) { e = Entry{}; }
}

void
UavcanAuthenticator::register_discovered(uint8_t nid, const char* type_name, uint8_t instance)
{
	const int bit = name_to_bit(type_name);
	if (bit < 0) return;
	if ( (_mask & (1u << bit)) == 0 ) return;

	int idx = find_slot_by_nid(nid);
	if (idx < 0) {
		idx = find_free_slot();
		if (idx < 0) idx = find_reclaimable_slot();

		_tab[idx] = Entry{};
		_tab[idx].node_id  = nid;
		_tab[idx].instance = instance;     // 0-based
		_tab[idx].type_bit = (int8_t)bit;
		_tab[idx].state    = State::NEW;
		_tab[idx].last_seen_time = hrt_absolute_time();
	} else {
		// Update instance if it changed; keep state as-is
		_tab[idx].instance = instance;     // still 0-based
		_tab[idx].type_bit = (int8_t)bit;
		_tab[idx].last_seen_time = hrt_absolute_time();
	}
}

void UavcanAuthenticator::refresh_node(uint8_t nid)
{
    int idx = find_slot_by_nid(nid);
    if (idx >= 0) {
	_tab[idx].last_seen_time = hrt_absolute_time();
    }
}

bool
UavcanAuthenticator::is_node_registered(uint8_t nid) const
{
	return find_slot_by_nid(nid) >= 0;
}

bool
UavcanAuthenticator::is_node_authed(uint8_t nid) const
{
	int idx = find_slot_by_nid(nid);
	return (idx >= 0) && (_tab[idx].state == State::OK);
}

void
UavcanAuthenticator::try_auth_node(uint8_t nid)
{
    	int idx = find_slot_by_nid(nid);
    	if (idx >= 0) try_auth_idx(static_cast<uint8_t>(idx));
}

void
UavcanAuthenticator::try_auth_idx(uint8_t idx)
{
    	if (idx >= kMaxEntries) return;		// check eligibility for SHA1Key request
	auto &e = _tab[idx];
	if (e.state == State::OK) return;	// already authed
    	if (e.attempts >= kMaxRetries) return;	// too many attempts

	const int rc = call_getsha1key(e.node_id);
	if (rc >= 0) {
		/* increment attempts only on call >= 0 to prevent
		 * burning retries on immediate transport errors*/
		e.last_request_time = hrt_absolute_time();
		e.attempts++;
    }
}

void
UavcanAuthenticator::on_auth_result(uint8_t server_nid, bool success)
{
    int idx = find_slot_by_nid(server_nid);
    if (idx < 0) return;
    _tab[idx].state = success ? State::OK : State::FAIL;
}

void
UavcanAuthenticator::remove_entry(uint8_t idx)
{
	if (idx >= kMaxEntries) return;
	_tab[idx] = Entry{};
}

UavcanAuthenticator::AuthCode UavcanAuthenticator::auth_status() const
{
	if (!_active)	return AuthCode::kInactive;
	if (is_armed())	return AuthCode::kArmed;

	// Enforce only in mode >= 2
	if (_mode < 2)	return AuthCode::kOk;

	bool any = false;
	for (const auto &e : _tab) {
		if (e.node_id == kInvalidNode) continue;
		any = true;
		if (e.state == State::FAIL) return AuthCode::kFailed;
		if (e.state != State::OK)   return AuthCode::kPending; // NEW or retrying
	}

	if (!any) return AuthCode::kNoDevices; // default policy: not an error
	return AuthCode::kOk;
}

bool
UavcanAuthenticator::auth_system_ok() const
{
	switch (auth_status()) {
	case AuthCode::kOk:
	case AuthCode::kInactive:
	case AuthCode::kArmed:
	case AuthCode::kNoDevices: // OK by default
		return true;
	case AuthCode::kPending:
	case AuthCode::kFailed:
		return false;
	}
	return true; // defensive
}

bool
UavcanAuthenticator::did_timeout(hrt_abstime last_seen) const
{
	return (hrt_absolute_time() - last_seen > kAuthTimeout);
}

void
UavcanAuthenticator::clear()
{
	reset_();
}

int
UavcanAuthenticator::call_getsha1key(uint8_t node_id)
{
	/* Lightweight wrapper so scheduling code does not need to know the
	 * details of the request layout. */
	com::h3robotics::GetSHA1Key::Request req;
	memcpy(&req.str_req[0], _request_payload, SHA1_LEN);

	return _getsha1_client.call(uavcan::NodeID(node_id), req);
}

void
UavcanAuthenticator::cb_getsha1key(const uavcan::ServiceCallResult<com::h3robotics::GetSHA1Key> &result)
{
	/* Shared service callback for the GetSHA1Key requests. Either funnels the
	 * response into the success path or triggers the retry logic. */

	// Ignore callbacks if authenticator is inactive
	if (!_active) {
		return;
	}
	/* Extract the node ID from the result and find the corresponding device from struct.*/
	on_auth_result(result.getCallID().server_node_id.get(), result.isSuccessful());
}
