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
 * @file authenticator.hpp
 *
 * UAVCAN Node Authentication.
 *
 * The PX4 UAVCAN driver keeps track of every remote node that appears on the
 * bus.  For certain device classes we additionally wish to verify that the
 * node belongs to our trusted fleet.  This class encapsulates that logic by
 * driving the vendor specific com.h3robotics.GetSHA1Key service, keeping track of
 * pending requests, retrying as needed and publishing the outcome so that the
 * commander pre-arm checks can reason about it.
 *
 * The method used here is a simplistic challenge-response based on SHA1 hashing.
 * In the future, SHA1 will be replaced with a more secure and modern method.
 * As for now, only the Smart Batteries and our custom payload modules support
 * SHA1 authentication. To implement such authentication with PX4 on CAN bus:
 *
 * We have defined a vendor-specific UAVCAN service com.h3robotics.255.GetSHA1Key that
 * sends a static challenge (the "cheesecake" seed) to the target node and expects
 * a SHA1 digest in response.  The content of the digest is not yet verified against
 * the shared secret. This will be implemented in future with device-specific unique codes.
 *
 * At the PX4 side, we have defined two parameters in the UAVCAN parameter group:
 * - UAVCAN_AUTH_MODE: 0 = disabled, 1 = request SHA1 keys, 2 = request and verify keys (WIP)
 * - UAVCAN_AUTH_MASK: Bitmask of device names that require authentication (in sync with sensor_bridge names)
 * UavcanAuthenticator class in the UAVCAN driver:
 * - spectates the active sensor bridges.
 * - registers the sensor bridge instances that require authentication.
 * - periodically issues GetSHA1Key requests to those nodes until a valid response is received
 * or the maximum number of retries is exhausted.
 * - maintains the authentication state of each node.
 *
 * @author Caglar Yilmaz <yilmaz.caglar@tubitak.gov.tr>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uavcan/uavcan.hpp>

/* We want to track if the drone is armed via related uORB topic.*/
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/uavcan_auth_status.h>

/* Vendor-specific UAVCAN service type.*/
#include <com/h3robotics/GetSHA1Key.hpp>

class UavcanAuthenticator : public ModuleParams
{
public:
	/* SHA1 Related Constants*/
	static constexpr uint8_t SHA1_LEN = 20;		///< SHA1 digest length
	static constexpr uint8_t kAuthSeedLen = SHA1_LEN;
	static const uint8_t kAuthSeed[kAuthSeedLen];
	/* Authentication node-wise state machine. Simplistic 3-state. */
	enum class State : uint8_t { NEW=0, OK=1, FAIL=2 };
	static constexpr hrt_abstime kAuthTimeout = 5'000'000; ///< 5 seconds timeout for response
	/* Helper variables for authentication management. */
	static constexpr uint8_t kMaxRetries = 3;	///< maximum authentication attempts per node
	static constexpr uint8_t kMaxEntries = 24;	///< amount of max authenticated nodes
	static constexpr uint8_t kInvalidNode = 0xFF;	///< Invalid Node ID marker
	static constexpr uint8_t kInvalidInst = 0xFF;	///< Invalid Instance marker
	static constexpr hrt_abstime kDelay = 250'000;	///< Delay between retries
	static constexpr hrt_abstime kPeriod = 100'000; ///< Minimum interval between update() calls
	/* Struct of component information that requires authentication. */
	struct Entry {
		uint8_t  node_id{ kInvalidNode };    	// 0..127 valid
		uint8_t  instance{ kInvalidInst };      // 0..(N-1), 0xFF = invalid
		int8_t   type_bit{ -1 };		// device type bit index, -1 = unassigned
		State    state{ State::NEW };		// authentication state
		uint8_t  attempts{ 0 };			// number of attempts made
		hrt_abstime last_request_time{ 0 };	// timestamp of last request
		hrt_abstime last_seen_time{ 0 };	// timestamp of last seen on bus.
	};
	/* A struct to inform external modules regarding authentication status. */
	enum class AuthCode : uint8_t {
		kOk = 0,          	// Not Enforcing.
		kInactive = 1,        	// Mode = 0
		kArmed = 2,           	// We never enforce while armed.
		kPending = 3,         	// Some registered nodes not OK yet (NEW / retrying).
		kFailed = 4,          	// At least one registered node explicitly FAIL.
		kNoDevices = 5        	// No registered nodes (policy: treat as OK by default).
	};

	/**
	 * Construct the authenticator.
	 * The UAVCAN node reference is shared with the rest of the driver and
	 * provides access to the dispatcher and memory pool.
	 */
	UavcanAuthenticator(uavcan::INode &node);

	/**
	 * Initialise the authenticator.
	 *
	 * The parameter UAVCAN_AUTH_MODE is already read in uavcan_main.cpp to trigger this
	 * code path. The reason we read it again here is to set up the internal state of
	 * the authenticator class, whether it compares received hashes to stored ones or not.
	 *
	 * The parameter UAVCAN_AUTH_MASK is also read to determine which device classes
	 * require authentication. It should be order-aligned and enumerated as per the sensor
	 * bridge device classes.
	 *
	 * The GetSHA1Key client is connected to the callback handler to process responses.
	 *
	 * @return PX4_OK
	 */
	int init();

	/**
	 * Periodic update call to advance pending requests.
	 *
	 * When the system is armed, authentication traffic is paused to avoid
	 * interfering with critical flight operations.  Requests are resumed
	 * when disarmed.
	 */
	void update();

	/** ----------- AUTHENTICATOR STATE MANAGEMENT ----------- **/
	/**
	 * Check whether the authenticator is active and drone is disarmed.
	 */
	bool auth_ready() const;

	/**
	 * Check whether the vehicle is currently armed.
	 *
	 * @return true if armed, false otherwise.
	 */
	bool is_armed() const;

	/**
	 * Update the arming state from uORB topic subscription.
	 *
	 * This function is called whenever the arming state changes.
	 */
	void set_armed();

	/**
	 * Get the overall authentication status.
	 *
	 * @return  AuthCode representing the current authentication status.
	 */
	AuthCode auth_status() const;

	/**
	 * Check whether all registered nodes are authenticated.
	 *
	 * TODO: Extend this to provide finer granularity,
	 * e.g. partial auth ok for critical devices auth to enable arming.
	 *
	 * @return  true if all registered nodes are authenticated, false otherwise.
	 */
	bool auth_system_ok() const;

	/**
	 * Get a bitmask of failed devices.
	 *
	 * Each bit in the returned mask corresponds to a device type
	 * that has failed authentication.
	 *
	 * @param system_ok  Overall system authentication status.
	 * @return           Bitmask of failed device types.
	 */
	uint32_t get_failed_devices_mask(bool system_ok) const;

	/** ----------- AUTHENTICATOR LIST MANAGEMENT ----------- **/
	/**
	 * Register a newly discovered node for authentication tracking.
	 *
	 * @param nid        Node ID of the discovered node.
	 * @param type_name  Bridge type name (for bitmask check).
	 * @param instance   Instance number of the bridge.
	 */
	void register_discovered(uint8_t nid, const char* type_name, uint8_t instance);

	/**
	 * Check whether a node ID is already registered.
	 *
	 * @param nid  	Node ID to check.
	 * @return     	true if the node ID is registered, false otherwise.
	 */
	bool is_node_registered(uint8_t nid) const;     // any entry with this nid?

	/**
	 * Check whether a node ID is authenticated.
	 *
	 * @param nid  Node ID to check.
	 * @return     true if the node ID is authenticated, false otherwise.
	 */
	bool is_node_authed(uint8_t nid) const;

	/**
	 * Refresh the last seen timestamp of a node.
	 *
	 * This function updates the last seen time of the specified node
	 * to the current time, indicating that the node is still active on the bus.
	 *
	 * @param nid  Node ID to refresh.
	 */
	void refresh_node(uint8_t nid);

	/**
	 * Check whether a node has timed out.
	 *
	 * A node is considered timed out if it has not been seen
	 * on the bus for a duration exceeding kAuthTimeout.
	 *
	 * @param last_seen  Last seen timestamp of the node.
	 * @return           true if the node has timed out, false otherwise.
	 */
	bool did_timeout(hrt_abstime last_seen) const;

	/**
	 * Remove an entry from the authentication table.
	 *
	 * @param idx  Index of the entry to remove.
	 */
	void remove_entry(uint8_t idx);

	/**
	 * Clear all authentication entries.
	 *
	 * This can be used to reset the authenticator state if necessary.
	 */
	void clear();

	/**
	 * Callback handler for authentication results. V0.1
	 *
	 * The success parameter indicates whether the service call was successful.
	 * We are going to check digest equality here in the future, but for now
	 * we just accept any successful response as authentication success.
	 *
	 * @param server_nid  Node ID of the server that responded.
	 * @param success     true if authentication succeeded, false otherwise.
	 */
	void on_auth_result(uint8_t server_nid, bool success);

	/** ----------- AUTHENTICATOR DECISION MANAGEMENT ----------- **/
	/**
	 * Check whether authentication is required for a given bridge type.
	 *
	 * Uses the UAVCAN_AUTH_MASK parameter to determine whether
	 * authentication is required for the specified bridge type.
	 *
	 * @param bridge_name  Bridge type name.
	 * @return             true if authentication is required, false otherwise.
	 */
	bool is_auth_required(const char* bridge_name) const;

	/**
	 * Attempt to authenticate a node by its node ID.
	 *
	 * @param nid  Node ID to authenticate.
	 */
	void try_auth_node(uint8_t nid);

private:
	uavcan::INode &_node;			///< UAVCAN node reference

	typedef uavcan::MethodBinder<UavcanAuthenticator *,
		void (UavcanAuthenticator::*)(const uavcan::ServiceCallResult<com::h3robotics::GetSHA1Key> &)> GetSHA1Callback;

	uavcan::ServiceClient<com::h3robotics::GetSHA1Key, GetSHA1Callback> _getsha1_client;

	uORB::Publication<uavcan_auth_status_s>	_auth_status_pub{ORB_ID(uavcan_auth_status)};
	uavcan_auth_status_s			_uavcan_auth_status{};		///< uORB message instance for authentication status publishing

	uORB::Subscription			_actuator_armed_sub{ORB_ID(actuator_armed)};

	/** ----------- UAVCAN DRIVER FUNCTIONS ----------- **/
	/**
	 * Callback handler for GetSHA1Key service responses.
	 *
	 * @param result  Service call result containing status and response data.
	 */
	void cb_getsha1key(const uavcan::ServiceCallResult<com::h3robotics::GetSHA1Key> &result);

	/**
	 * Call the GetSHA1Key service for a specific node.
	 *
	 * @param node_id  Node ID to authenticate.
	 * @return        0 on success, negative error code on failure.
	 */
	int call_getsha1key(uint8_t node_id);

	/** ----------- AUTHENTICATOR HELPERS ----------- **/
	/**
	 * Map a bridge type name to its corresponding bit index.
	 *
	 * @param name  Bridge type name.
	 * @return      Bit index for the bridge type, or -1 if not found.
	 */
	static int name_to_bit(const char* name);

	/**
	 * Find the slot index for a given node ID.
	 *
	 * @param nid  Node ID to search for.
	 * @return     Index of the slot if found, -1 otherwise.
	 */
	int find_slot_by_nid(uint8_t nid) const;

	/**
	 * Find a free slot in the authentication table.
	 *
	 * @return  Index of the free slot if available, -1 otherwise.
	 */
	int find_free_slot() const;

	/**
	 * Find a reclaimable slot in the authentication table.
	 *
	 * A reclaimable slot is one that has failed authentication
	 * and has exceeded the retry timeout.
	 *
	 * @return  Index of the reclaimable slot if available, -1 otherwise.
	 */
	int find_reclaimable_slot() const;

	/**
	 * Try to authenticate a node by its index in the table.
	 *
	 * @param idx  Index of the node in the authentication table.
	 */
	void try_auth_idx(uint8_t idx);

	/**
	 * Reset the authenticator state.
	 *
	 * Clears all entries in the authentication table and resets internal state.
	 */
	void reset_();

	/** ----------- Private Members ----------- **/
	uint8_t _request_payload[SHA1_LEN] {}; 	///< pre-computed request payload
	Entry    _tab[kMaxEntries];		///< tracked nodes table
	uint32_t _mask{0};			///< device type bitmask from parameter
	int32_t _mode{0};			///< authentication mode from parameter | must match with parameter type definition int32_t
	bool _active{false};			///< whether the authenticator is active
	bool _armed{false};			///< whether the vehicle is currently armed
	hrt_abstime _last_update_time{0};	///< last update timestamp for rate limiting

};
