#include "loadcell.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanLoadCellBridge::NAME = "loadcell";

UavcanLoadCellBridge::UavcanLoadCellBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_loadcell", ORB_ID(sensor_load_cell)),
	_sub_loadcell(node)
{
}

int
UavcanLoadCellBridge::init()
{
	int res = _sub_loadcell.start(LoadCellCbBinder(this, &UavcanLoadCellBridge::loadcell_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanLoadCellBridge::loadcell_sub_cb(const uavcan::ReceivedDataStructure<com::h3robotics::LoadCellArray> &msg)
{
	sensor_load_cell_s loadcell{};
	loadcell.timestamp = uavcan::UtcTime(msg.timestamp).toUSec();

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_UAVCAN;
	device_id.devid_s.bus = 0;
	device_id.devid_s.devtype = DRV_LOAD_CELL_DEVTYPE_UAVCAN;
	device_id.devid_s.address = msg.getSrcNodeID().get() & 0xFF;

	loadcell.device_id = device_id.devid;

	for (unsigned i = 0; i < 4; i++) {
		loadcell.raw[i] = static_cast<float>(msg.load_cells[i]);
	}

	publish(msg.getSrcNodeID().get(), &loadcell);
}
