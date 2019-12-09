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

struct YawSimulatorPacket_t {
	double simYaw, simPitch, simRoll;
	double gameYaw, gamePitch, gameRoll;
	double motorSpeed1, motorSpeed2, motorSpeed3;
	double batteryLevel;

	std::string getString();
};

// forward declarations


class YawSimulatorClient {
public:
	void init();
	void shutdown();

	void setYawSimulatorIPAddress(const std::string& ipAddress);
	void connect() { _udpClientThreadShouldConnect = true; }
	void disconnect() { _udpClientThreadShouldConnect = false; }
	bool isConnected() { return _udpClientThreadConnected; }

	YawSimulatorPacket_t getLastPacket();
	vr::HmdQuaternion_t getSimRotation();

private:
	static void _udpClientThreadFunc(YawSimulatorClient* _this);
	static bool parsePacket(const char* buffer, YawSimulatorPacket_t& yawSimulatorPacket);

	std::string _ipAddress;
	std::thread _udpClientThread;
	volatile bool _udpClientThreadRunning = false;
	volatile bool _udpClientThreadStopFlag = false;
	volatile bool _udpClientThreadShouldConnect = false;
	volatile bool _udpClientThreadConnected = false;
	boost::posix_time::ptime _nextConnectionAttemptTime;
	std::mutex _mutex;
	YawSimulatorPacket_t m_lastPacket;
	boost::posix_time::ptime _lastLogTime;
};


} // end namespace driver
} // end namespace vrinputemulator
#endif