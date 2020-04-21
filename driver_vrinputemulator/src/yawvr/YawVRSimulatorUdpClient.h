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

struct YawVRSimulatorPacket_t {
	double simYaw, simPitch, simRoll;
	double gameYaw, gamePitch, gameRoll;
	double motorSpeed1, motorSpeed2, motorSpeed3;
	double batteryLevel;

	std::string getString();
};

// forward declarations


class YawVRSimulatorClient {
public:
	void init();
	void shutdown();

	void checkYawVRGameEngineConfig();
	bool isYawVRGameEngineOverriden() { return _yawVRGameEngineOverride; }
	void yawVRGameEngineOverride(bool override);
	bool isYawVRGameEngine3dofModeEnabled() { return _yawVRGameEngine3dofModeEnabled; }
	void enableYawVRGameEngine3dofMode(bool enable);
	std::string& getYawVRSimulatorIPAddress() { return _ipAddress; }
	void setYawVRSimulatorIPAddress(const std::string& ipAddress);
	bool getShouldConnect() { return ((uint8_t)_connectionState & (uint8_t)ConnectionState::ShouldConnect); }
	void shouldConnect(bool connect) {
		_connectionState = (ConnectionState)(((uint8_t)_connectionState & ~(uint8_t)ConnectionState::ShouldConnect) |
			(uint8_t)(connect ? ConnectionState::ShouldConnect : ConnectionState::Disconnected) |
			((uint8_t)ConnectionState::Dirty));
	}
	bool isConnected() { return ((uint8_t)_connectionState & (uint8_t)ConnectionState::Connected); }

	YawVRSimulatorPacket_t getLastPacket();
	vr::HmdQuaternion_t getSimRotation();

private:
	static void _udpClientThreadFunc(YawVRSimulatorClient* _this);
	static bool parsePacket(const char* buffer, YawVRSimulatorPacket_t& yawVRSimulatorPacket);

	bool _yawVRGameEngineOverride;
	bool _yawVRGameEngine3dofModeEnabled;
	std::string _ipAddress;
	std::thread _udpClientThread;
	boost::posix_time::ptime _lastYawVRGameEngineCheckTime;
	time_t _yawVRGameEngineCfgFileLastWriteTime;
	volatile bool _udpClientThreadRunning = false;
	volatile bool _udpClientThreadStopFlag = false;
	enum class ConnectionState : uint8_t {
		Disconnected = 0x00,
		Connected = 0x01,
		ShouldConnect = Connected << 1,
		Dirty = ShouldConnect << 1
	};
	volatile ConnectionState _connectionState = ConnectionState::Disconnected;
	boost::posix_time::ptime _nextConnectionAttemptTime;
	std::mutex _mutex;
	YawVRSimulatorPacket_t m_lastPacket;
	boost::posix_time::ptime _lastLogTime;
};


} // end namespace driver
} // end namespace vrinputemulator
#endif
