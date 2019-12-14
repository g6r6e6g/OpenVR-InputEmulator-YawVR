#ifdef YAWVRUNITYTESTER
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

struct YawVRUnityTesterPacket_t {
	vr::HmdVector3d_t simYawPitchRoll;
	vr::HmdVector3d_t mcZeroPos;
	vr::HmdQuaternion_t mcZeroRot;
	vr::HmdVector3d_t mcRefPos;
	vr::HmdQuaternion_t mcRotDiff;
	vr::HmdVector3d_t hmdPos;
	vr::HmdQuaternion_t hmdRot;
	vr::HmdVector3d_t ctrlr1Pos;
	vr::HmdQuaternion_t ctrlr1Rot;
	vr::HmdVector3d_t ctrlr2Pos;
	vr::HmdQuaternion_t ctrlr2Rot;
	vr::HmdVector3d_t tref1Pos;
	vr::HmdQuaternion_t tref1Rot;
	vr::HmdVector3d_t tref2Pos;
	vr::HmdQuaternion_t tref2Rot;
	vr::HmdVector3d_t mcHmdPos;
	vr::HmdQuaternion_t mcHmdRot;

	std::string getString();
};

// forward declarations


class YawVRUnityTesterUdpClient {
public:
	void init();
	void shutdown();

	YawVRUnityTesterPacket_t* lockPacket();
	void unlockPacket();
	std::string getPacket() const;

private:
	static void _udpClientThreadFunc(YawVRUnityTesterUdpClient* _this);

	std::thread _udpClientThread;
	volatile bool _udpClientThreadRunning = false;
	volatile bool _udpClientThreadStopFlag = false;
	boost::posix_time::ptime m_nextConnectionAttemptTime;
	std::mutex _mutex;
	YawVRUnityTesterPacket_t m_lastPacket;
	boost::posix_time::ptime m_lastPacketSentTime;
};


} // end namespace driver
} // end namespace vrinputemulator
#endif
