#pragma once

#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <openvr_driver.h>
#include <vrinputemulator_types.h>
#include <openvr_math.h>

#pragma comment(lib, "ws2_32.lib")

// driver namespace
namespace vrinputemulator {

// forward declarations

namespace driver {

struct YawVRPacket_t {
	double simYaw, simPitch, simRoll;
	double gameYaw, gamePitch, gameRoll;
	double motorSpeed1, motorSpeed2, motorSpeed3;
	double batteryLevel;

	std::string getString();
};

// forward declarations


class YawVRUdpClient {
public:
	void init();
	void shutdown();

	const YawVRPacket_t& getLastPacket() const;
	vr::HmdQuaternion_t getSimRotation() const;

private:
	static void _udpClientThreadFunc(YawVRUdpClient* _this);
	static bool parsePacket(const char* buffer, YawVRPacket_t& yawVRPacket);

	std::thread _udpClientThread;
	volatile bool _udpClientThreadRunning = false;
	volatile bool _udpClientThreadStopFlag = false;
	boost::posix_time::ptime m_nextConnectionAttemptTime;
	YawVRPacket_t m_lastPacket;
};


} // end namespace driver
} // end namespace vrinputemulator
