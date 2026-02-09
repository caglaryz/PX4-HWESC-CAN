#pragma once

#include "sensor_bridge.hpp"

#include <stdint.h>

#include <uORB/topics/sensor_load_cell.h>

#include <com/h3robotics/LoadCellArray.hpp>

class UavcanLoadCellBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanLoadCellBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	void loadcell_sub_cb(const uavcan::ReceivedDataStructure<com::h3robotics::LoadCellArray> &msg);

	typedef uavcan::MethodBinder < UavcanLoadCellBridge *,
		void (UavcanLoadCellBridge::*)
		(const uavcan::ReceivedDataStructure<com::h3robotics::LoadCellArray> &) >
		LoadCellCbBinder;

	uavcan::Subscriber<com::h3robotics::LoadCellArray, LoadCellCbBinder> _sub_loadcell;

};
