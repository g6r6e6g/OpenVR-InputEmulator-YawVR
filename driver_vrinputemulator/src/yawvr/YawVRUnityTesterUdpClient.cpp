#include "YawVRUnityTesterUdpClient.h"

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include "../driver/ServerDriver.h"

#define YAWVRUNITYTESTER_UDP_CLIENT	"127.0.0.1"
#define YAWVRUNITYTESTER_UDP_PORT	28888
#define YAWVR_CNX_ATTEMPT_DELAY_SEC	5
#define YAWVR_CNX_PACKET_SEND_FREQ	60.0f
#define YAWVRUNITYTESTER_UDP_PACKET	"SY[%.03f]SP[%.03f]SR[%.03f]\
MCZPX[%.03f]MCZPY[%.03f]MCZPZ[%.03f]MCZRX[%.03f]MCZRY[%.03f]MCZRZ[%.03f]MCZRW[%.03f]\
MCRPX[%.03f]MCRPY[%.03f]MCRPZ[%.03f]MCRDX[%.03f]MCRDY[%.03f]MCRDZ[%.03f]MCRDW[%.03f]\
HPX[%.03f]HPY[%.03f]HPZ[%.03f]HRX[%.03f]HRY[%.03f]HRZ[%.03f]HRW[%.03f]\
C1PX[%.03f]C1PY[%.03f]C1PZ[%.03f]C1RX[%.03f]C1RY[%.03f]C1RZ[%.03f]C1RW[%.03f]\
C2PX[%.03f]C2PY[%.03f]C2PZ[%.03f]C2RX[%.03f]C2RY[%.03f]C2RZ[%.03f]C2RW[%.03f]\
T1PX[%.03f]T1PY[%.03f]T1PZ[%.03f]T1RX[%.03f]T1RY[%.03f]T1RZ[%.03f]T1RW[%.03f]\
T2PX[%.03f]T2PY[%.03f]T2PZ[%.03f]T2RX[%.03f]T2RY[%.03f]T2RZ[%.03f]T2RW[%.03f]"

namespace vrinputemulator {
namespace driver {

std::string YawVRUnityTesterPacket_t::getString() {
	return boost::str(boost::format("YawVRUnityTesterPacket(simYawPitchRoll:(%.3f, %.3f, %.3f),\
 mcZeroPos:(%.3f, %.3f, %.3f), mcZeroRot:(%.3f, %.3f, %.3f, %.3f),\
 mcRefPos:(%.3f, %.3f, %.3f), mcRotDiff:(%.3f, %.3f, %.3f, %.3f),\
 hmdPos:(%.3f, %.3f, %.3f), hmdRot:(%.3f, %.3f, %.3f, %.3f),\
 ctrlr1Pos:(%.3f, %.3f, %.3f), ctrlr1Rot:(%.3f, %.3f, %.3f, %.3f),\
 ctrlr2Pos:(%.3f, %.3f, %.3f), ctrlr2Rot:(%.3f, %.3f, %.3f, %.3f),\
 tref1Pos:(%.3f, %.3f, %.3f), tref1Rot:(%.3f, %.3f, %.3f, %.3f),\
 tref2Pos:(%.3f, %.3f, %.3f), tref2Rot:(%.3f, %.3f, %.3f, %.3f))") %
		simYawPitchRoll.v[0] % simYawPitchRoll.v[1] % simYawPitchRoll.v[2] %
		mcZeroPos.v[0] % mcZeroPos.v[1] % mcZeroPos.v[2] %
		mcZeroRot.x % mcZeroRot.y % mcZeroRot.z% mcZeroRot.w %
		mcRefPos.v[0] % mcRefPos.v[1] % mcRefPos.v[2] %
		mcRotDiff.x % mcRotDiff.y % mcRotDiff.z% mcRotDiff.w %
		hmdPos.v[0] % hmdPos.v[1] % hmdPos.v[2] %
		hmdRot.x % hmdRot.y % hmdRot.z% hmdRot.w %
		ctrlr1Pos.v[0] % ctrlr1Pos.v[1] % ctrlr1Pos.v[2] %
		ctrlr1Rot.x % ctrlr1Rot.y % ctrlr1Rot.z% ctrlr1Rot.w %
		ctrlr2Pos.v[0] % ctrlr2Pos.v[1] % ctrlr2Pos.v[2] %
		ctrlr2Rot.x % ctrlr2Rot.y % ctrlr2Rot.z% ctrlr2Rot.w %
		tref1Pos.v[0] % tref1Pos.v[1] % tref1Pos.v[2] %
		tref1Rot.x % tref1Rot.y % tref1Rot.z% tref1Rot.w %
		tref2Pos.v[0] % tref2Pos.v[1] % tref2Pos.v[2] %
		tref2Rot.x % tref2Rot.y % tref2Rot.z% tref2Rot.w);
}

void YawVRUnityTesterUdpClient::init() {
	LOG(TRACE) << "YawVRUnityTesterUdpClient::init";
	_udpClientThreadStopFlag = false;
	_udpClientThread = std::thread(_udpClientThreadFunc, this);
	LOG(DEBUG) << "YawVRUnityTesterUdpClient::init: thread created";
	m_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time();
	m_lastPacketSentTime = boost::posix_time::microsec_clock::universal_time();
}

void YawVRUnityTesterUdpClient::shutdown() {
	LOG(TRACE) << "YawVRUnityTesterUdpClient::shutdown";
	if (_udpClientThreadRunning) {
		_udpClientThreadStopFlag = true;
		LOG(DEBUG) << "YawVRUnityTesterUdpClient::shutdown: waiting end of thread (joining)";
		_udpClientThread.join();
	}
}

YawVRUnityTesterPacket_t* YawVRUnityTesterUdpClient::lockPacket() {
	_mutex.lock();
	return &m_lastPacket;
}

void YawVRUnityTesterUdpClient::unlockPacket() {
	_mutex.unlock();
}

std::string YawVRUnityTesterUdpClient::getPacket() const {
	return boost::str(boost::format(YAWVRUNITYTESTER_UDP_PACKET) %
		m_lastPacket.simYawPitchRoll.v[0] % m_lastPacket.simYawPitchRoll.v[1] % m_lastPacket.simYawPitchRoll.v[2] %
		m_lastPacket.mcZeroPos.v[0] % m_lastPacket.mcZeroPos.v[1] % m_lastPacket.mcZeroPos.v[2] %
		m_lastPacket.mcZeroRot.x % m_lastPacket.mcZeroRot.y % m_lastPacket.mcZeroRot.z% m_lastPacket.mcZeroRot.w %
		m_lastPacket.mcRefPos.v[0] % m_lastPacket.mcRefPos.v[1] % m_lastPacket.mcRefPos.v[2]%
		m_lastPacket.mcRotDiff.x % m_lastPacket.mcRotDiff.y % m_lastPacket.mcRotDiff.z % m_lastPacket.mcRotDiff.w);
}

void YawVRUnityTesterUdpClient::_udpClientThreadFunc(YawVRUnityTesterUdpClient* _this) {
	SOCKET sockfd = INVALID_SOCKET;
	SOCKADDR_IN sockAddr;
	int addrLen = sizeof(sockAddr);
	char sndBuffer[YAWVRUNITYTESTER_UDP_PORT];
	int sndSize;

	_this->_udpClientThreadRunning = true;
	LOG(DEBUG) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: thread started";
	try {
		while (!_this->_udpClientThreadStopFlag) {
			try {
				if (sockfd == INVALID_SOCKET && boost::posix_time::microsec_clock::universal_time() > _this->m_nextConnectionAttemptTime) {
					LOG(TRACE) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: Connecting to YawVR Unity Tester (port " << YAWVRUNITYTESTER_UDP_PORT << ") ...";
					sockfd = socket(AF_INET, SOCK_DGRAM, 0);
					if (sockfd == INVALID_SOCKET) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not connect to YawVR Unity Tester : Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: Socket created.";
					int enable = 1;
					if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&enable), sizeof(int)) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not set reuseaddr socket option: Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: Timeout socket option set.";
					sockAddr.sin_family = AF_INET;
					sockAddr.sin_port = htons(YAWVRUNITYTESTER_UDP_PORT);
					sockAddr.sin_addr.s_addr = inet_addr(YAWVRUNITYTESTER_UDP_CLIENT);
				}
				if (sockfd != INVALID_SOCKET && boost::posix_time::microsec_clock::universal_time() > _this->m_lastPacketSentTime + boost::posix_time::milliseconds(1000*int(1.0f/YAWVR_CNX_PACKET_SEND_FREQ))) {
					LOG(DEBUG) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: sending packet : " << _this->m_lastPacket.getString();
					_this->m_lastPacketSentTime = boost::posix_time::microsec_clock::universal_time();
					std::string packet = _this->getPacket();
					strcpy_s(sndBuffer, packet.c_str());
					sndBuffer[packet.length()] = '\0';
#pragma warning(push)
#pragma warning(disable:4267)
					sndSize = sendto(sockfd, (const char *)sndBuffer, strlen(sndBuffer), 0, (SOCKADDR*)&sockAddr, (int)addrLen);
#pragma warning(pop)
					if (sndSize == SOCKET_ERROR) {
						int err = WSAGetLastError();
						LOG(ERROR) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: could not send packet : " << sndBuffer << "\nError code %d." << err;
					}
				}
			} catch (std::exception& ex) {
				if (sockfd != INVALID_SOCKET) {
					closesocket(sockfd);
					sockfd = INVALID_SOCKET;
				}
				LOG(ERROR) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: Exception caught in YawVR Unity Tester UDP Client receive loop !\n" << ex.what() << "\nNew attempt in " << YAWVR_CNX_ATTEMPT_DELAY_SEC << " seconds ...";
				_this->m_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds(YAWVR_CNX_ATTEMPT_DELAY_SEC*1000);
			}
			if (sockfd == INVALID_SOCKET) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			}
		}
		if (sockfd != INVALID_SOCKET) {
			closesocket(sockfd);
			sockfd = INVALID_SOCKET;
		}
	} catch (std::exception& ex) {
		LOG(ERROR) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: Exception caught in YawVR Unity Tester UDP Client thread: " << ex.what();
	}
	_this->_udpClientThreadRunning = false;
	LOG(DEBUG) << "YawVRUnityTesterUdpClient::_udpClientThreadFunc: thread stopped";
}


} // end namespace driver
} // end namespace vrinputemulator
