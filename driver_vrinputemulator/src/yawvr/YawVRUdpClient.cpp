#include "YawVRUdpClient.h"

#include <winsock2.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include "../driver/ServerDriver.h"

#define YAWVR_UDP_PORT				28067
#define YAWVR_CNX_ATTEMPT_DELAY_SEC	5		
#define YAWVR_UDP_PACKET_MAXSIZE	256
#define YAWVR_UDP_PACKET_SIMYAW		"simYaw"
#define YAWVR_UDP_PACKET_SIMPITCH	"simPitch"
#define YAWVR_UDP_PACKET_SIMROLL	"simRoll"
#define YAWVR_UDP_PACKET_GAMEYAW	"gameYaw"
#define YAWVR_UDP_PACKET_GAMEPITCH	"gamePitch"
#define YAWVR_UDP_PACKET_GAMEROLL	"gameRoll"
#define YAWVR_UDP_PACKET_MOTORSPD1	"motorSpd1"
#define YAWVR_UDP_PACKET_MOTORSPD2	"motorSpd2"
#define YAWVR_UDP_PACKET_MOTORSPD3	"motorSpd3"
#define YAWVR_UDP_PACKET_BATTERYLVL	"batteryLvl"
#define YAWVR_UDP_PACKET_REGEX		"^(?:SY\\[(?<" ## YAWVR_UDP_PACKET_SIMYAW ## ">[+-]?\\d+[\\.]?\\d*)])?(?:SP\\[(?<" ## YAWVR_UDP_PACKET_SIMPITCH ## ">[+-]?\\d+[\\.]?\\d*)])?(?:SR\\[(?<" ## YAWVR_UDP_PACKET_SIMROLL ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GY\\[(?<" ## YAWVR_UDP_PACKET_GAMEYAW ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GP\\[(?<" ## YAWVR_UDP_PACKET_GAMEPITCH ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GR\\[(?<" ## YAWVR_UDP_PACKET_GAMEROLL ## ">[+-]?\\d+[\\.]?\\d*)])?(?:S1\\[(?<" ## YAWVR_UDP_PACKET_MOTORSPD1 ## ">\\d+[\\.]?\\d*)\\])?(?:S2\\[(?<" ## YAWVR_UDP_PACKET_MOTORSPD2 ## ">\\d+[\\.]?\\d*)\\])?(?:S3\\[(?<" ## YAWVR_UDP_PACKET_MOTORSPD3 ## ">\\d+[\\.]?\\d*)\\])?(?:U\\[(?<" ## YAWVR_UDP_PACKET_BATTERYLVL ## ">\\d+[\\.]?\\d*)\\])?.*$"

namespace vrinputemulator {
namespace driver {

std::string YawVRPacket_t::getString() {
	return boost::str(boost::format("YawVRPacket(simYaw:%.3f, simPitch:%.3f, simRoll:%.3f, gameYaw:%.3f, gamePitch:%.3f, gameRoll:%.3f, motorSpeed1:%.3f, motorSpeed3:%.3f, motorSpeed3:%.3f, batteryLevel:%.3f)") %
		simYaw % simPitch % simRoll % gameYaw % gamePitch % gameRoll % motorSpeed1 % motorSpeed2 % motorSpeed3 % batteryLevel);
}

void YawVRUdpClient::init() {
	LOG(TRACE) << "YawVRUdpClient::init";
	_udpClientThreadStopFlag = false;
	_udpClientThread = std::thread(_udpClientThreadFunc, this);
	LOG(DEBUG) << "YawVRUdpClient::init: thread created";
	m_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time();
}

void YawVRUdpClient::shutdown() {
	LOG(TRACE) << "YawVRUdpClient::shutdown";
	if (_udpClientThreadRunning) {
		_udpClientThreadStopFlag = true;
		LOG(DEBUG) << "YawVRUdpClient::shutdown: waiting end of thread (joining)";
		_udpClientThread.join();
	}
}

const YawVRPacket_t& YawVRUdpClient::getLastPacket() const {
	return m_lastPacket;
}

vr::HmdQuaternion_t YawVRUdpClient::getSimRotation() const {
	return vrmath::quaternionFromYawPitchRoll(-m_lastPacket.simYaw*M_PI/180.0, m_lastPacket.simPitch*M_PI/180.0, -m_lastPacket.simRoll*M_PI/180.0);
}

bool YawVRUdpClient::parsePacket(const char *buffer, YawVRPacket_t& yawVRPacket) {
	boost::regex packetRegEx(YAWVR_UDP_PACKET_REGEX);
	boost::smatch match;
	std::string bufferStr(buffer);
	if (boost::regex_search(bufferStr, match, packetRegEx)) {
		yawVRPacket.simYaw = match[YAWVR_UDP_PACKET_SIMYAW].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_SIMYAW]);
		yawVRPacket.simPitch = match[YAWVR_UDP_PACKET_SIMPITCH].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_SIMPITCH]);
		yawVRPacket.simRoll = match[YAWVR_UDP_PACKET_SIMROLL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_SIMROLL]);
		yawVRPacket.gameYaw = match[YAWVR_UDP_PACKET_GAMEYAW].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_GAMEYAW]);
		yawVRPacket.gamePitch = match[YAWVR_UDP_PACKET_GAMEPITCH].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_GAMEPITCH]);
		yawVRPacket.gameRoll = match[YAWVR_UDP_PACKET_GAMEROLL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_GAMEROLL]);
		yawVRPacket.motorSpeed1 = match[YAWVR_UDP_PACKET_MOTORSPD1].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_MOTORSPD1]);
		yawVRPacket.motorSpeed2 = match[YAWVR_UDP_PACKET_MOTORSPD2].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_MOTORSPD2]);
		yawVRPacket.motorSpeed3 = match[YAWVR_UDP_PACKET_MOTORSPD3].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_MOTORSPD3]);
		yawVRPacket.batteryLevel = match[YAWVR_UDP_PACKET_BATTERYLVL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVR_UDP_PACKET_BATTERYLVL]);
		return true;
	}
	return false;
}

void YawVRUdpClient::_udpClientThreadFunc(YawVRUdpClient* _this) {
	SOCKET sockfd = INVALID_SOCKET;
	SOCKADDR_IN sockAddr;
	int addrLen = sizeof(sockAddr);
	char rcvBuffer[YAWVR_UDP_PORT];
	int rcvSize;

	_this->_udpClientThreadRunning = true;
	LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: thread started";
	try {
		while (!_this->_udpClientThreadStopFlag) {
			try {
				if (sockfd == INVALID_SOCKET && boost::posix_time::microsec_clock::universal_time() > _this->m_nextConnectionAttemptTime) {
					LOG(TRACE) << "YawVRUdpClient::_udpClientThreadFunc: Connecting to YawVR (port " << YAWVR_UDP_PORT << ") ...";
					sockfd = socket(AF_INET, SOCK_DGRAM, 0);
					if (sockfd == INVALID_SOCKET) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not connect to YawVR : Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: Socket created.";
					struct timeval readTimeout;
					readTimeout.tv_sec = 100;
					readTimeout.tv_usec = 0;
					if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&readTimeout), sizeof readTimeout) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not set timeout socket option: Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: Timeout socket option set.";
					sockAddr.sin_family = AF_INET;
					sockAddr.sin_port = htons(YAWVR_UDP_PORT);
					sockAddr.sin_addr.s_addr = INADDR_ANY;
					if (bind(sockfd, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not bind to YawVR : Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: Socket bound.";
				}
				if (sockfd != INVALID_SOCKET) {
					rcvSize = recvfrom(sockfd, rcvBuffer, sizeof(rcvBuffer) - 1, 0, (SOCKADDR*)&sockAddr, &addrLen);
					if (rcvSize != SOCKET_ERROR) {
						rcvBuffer[rcvSize] = '\0';
						try {
							if (parsePacket(rcvBuffer, _this->m_lastPacket)) {
								LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: received packet : " << _this->m_lastPacket.getString();
							}
							else {
								LOG(ERROR) << "YawVRUdpClient::_udpClientThreadFunc: could not parse received packet : " << rcvBuffer;
							}
						} catch (std::exception& ex) {
							LOG(ERROR) << "YawVRUdpClient::_udpClientThreadFunc: could not parse received packet : " << rcvBuffer << "\n" << ex.what();
						}
					}
					else {
						int err = WSAGetLastError();
						if (err != WSAETIMEDOUT)
							throw std::runtime_error(boost::str(boost::format("Could not receive from YawVR: Error code %d.") % err));
					}
				}
			} catch (std::exception& ex) {
				if (sockfd != INVALID_SOCKET) {
					closesocket(sockfd);
					sockfd = INVALID_SOCKET;
				}
				LOG(ERROR) << "YawVRUdpClient::_udpClientThreadFunc: Exception caught in YawVR UDP client receive loop !\n" << ex.what() << "\nNew attempt in " << YAWVR_CNX_ATTEMPT_DELAY_SEC << " seconds ...";
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
		LOG(ERROR) << "YawVRUdpClient::_udpClientThreadFunc: Exception caught in YawVR UDP client thread: " << ex.what();
	}
	_this->_udpClientThreadRunning = false;
	LOG(DEBUG) << "YawVRUdpClient::_udpClientThreadFunc: thread stopped";
}


} // end namespace driver
} // end namespace vrinputemulator
