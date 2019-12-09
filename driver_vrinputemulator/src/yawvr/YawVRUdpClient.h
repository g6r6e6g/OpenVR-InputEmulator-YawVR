#ifdef YAWVR
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


class YawVRUdpClient { //TORENAME YawSimulatorClient
public:
	void init();
	void shutdown();

	void setYawSimulatorIPAddress(const std::string& ipAddress);

	YawVRPacket_t getLastPacket();
	vr::HmdQuaternion_t getSimRotation();

private:
	static void _udpClientThreadFunc(YawVRUdpClient* _this);
	static bool parsePacket(const char* buffer, YawVRPacket_t& yawVRPacket);

	std::string _ipAddress;
	std::thread _udpClientThread;
	volatile bool _udpClientThreadRunning = false;
	volatile bool _udpClientThreadStopFlag = false;
	volatile bool _udpClientThreadShouldDisconnect = false;
	boost::posix_time::ptime _nextConnectionAttemptTime;
	boost::posix_time::ptime _lastLogTime;
	std::mutex _mutex;
	YawVRPacket_t m_lastPacket;
};


} // end namespace driver
} // end namespace vrinputemulator
#endif
