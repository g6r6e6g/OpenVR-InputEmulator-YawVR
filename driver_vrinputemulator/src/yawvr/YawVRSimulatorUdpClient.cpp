#ifdef YAWVR
#include "YawVRSimulatorUdpClient.h"

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
//#include <Ws2tcpip.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include "../driver/ServerDriver.h"

/*TODEL #if YAWVRSIM_EMULATOR
#define YAWVRSIM_TCP_IP						"192.168.1.18" // YawVR emulator, just launch the Unity Emulator app
#else
#define YAWVRSIM_TCP_IP						"192.168.1.28" // Real YawVR simulator device, start the Android appand start the device
#endif*/
#define YAWVRSIM_TCP_PORT					50020
#define YAWVRSIM_UDP_PORT					28067
#define YAWVRSIM_UDP_PORT_CHECKIN_GAME_NAME	"YawVRMotionCompensation"
#define YAWVRSIM_CNX_ATTEMPT_DELAY_SEC		5
#define YAWVRSIM_UDP_PACKET_MAXSIZE			256
#define YAWVRSIM_UDP_PACKET_SIMYAW			"simYaw"
#define YAWVRSIM_UDP_PACKET_SIMPITCH		"simPitch"
#define YAWVRSIM_UDP_PACKET_SIMROLL			"simRoll"
#define YAWVRSIM_UDP_PACKET_GAMEYAW			"gameYaw"
#define YAWVRSIM_UDP_PACKET_GAMEPITCH		"gamePitch"
#define YAWVRSIM_UDP_PACKET_GAMEROLL		"gameRoll"
#define YAWVRSIM_UDP_PACKET_MOTORSPD1		"motorSpd1"
#define YAWVRSIM_UDP_PACKET_MOTORSPD2		"motorSpd2"
#define YAWVRSIM_UDP_PACKET_MOTORSPD3		"motorSpd3"
#define YAWVRSIM_UDP_PACKET_BATTERYLVL		"batteryLvl"
#define YAWVRSIM_UDP_PACKET_REGEX			"^(?:SY\\[(?<" ## YAWVRSIM_UDP_PACKET_SIMYAW ## ">[+-]?\\d+[\\.]?\\d*)])?(?:SP\\[(?<" ## YAWVRSIM_UDP_PACKET_SIMPITCH ## ">[+-]?\\d+[\\.]?\\d*)])?(?:SR\\[(?<" ## YAWVRSIM_UDP_PACKET_SIMROLL ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GY\\[(?<" ## YAWVRSIM_UDP_PACKET_GAMEYAW ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GP\\[(?<" ## YAWVRSIM_UDP_PACKET_GAMEPITCH ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GR\\[(?<" ## YAWVRSIM_UDP_PACKET_GAMEROLL ## ">[+-]?\\d+[\\.]?\\d*)])?(?:S1\\[(?<" ## YAWVRSIM_UDP_PACKET_MOTORSPD1 ## ">\\d+[\\.]?\\d*)\\])?(?:S2\\[(?<" ## YAWVRSIM_UDP_PACKET_MOTORSPD2 ## ">\\d+[\\.]?\\d*)\\])?(?:S3\\[(?<" ## YAWVRSIM_UDP_PACKET_MOTORSPD3 ## ">\\d+[\\.]?\\d*)\\])?(?:U\\[(?<" ## YAWVRSIM_UDP_PACKET_BATTERYLVL ## ">\\d+[\\.]?\\d*)\\])?.*$"

namespace vrinputemulator {
namespace driver {

std::string YawVRSimulatorPacket_t::getString() {
	return boost::str(boost::format("YawVRSimulatorPacket(simYaw:%.3f, simPitch:%.3f, simRoll:%.3f, gameYaw:%.3f, gamePitch:%.3f, gameRoll:%.3f, motorSpeed1:%.3f, motorSpeed3:%.3f, motorSpeed3:%.3f, batteryLevel:%.3f)") %
		simYaw % simPitch % simRoll % gameYaw % gamePitch % gameRoll % motorSpeed1 % motorSpeed2 % motorSpeed3 % batteryLevel);
}

void YawVRSimulatorClient::init() {
	LOG(TRACE) << "YawVRSimulatorUdpClient::init";
	_udpClientThreadStopFlag = false;
	_udpClientThread = std::thread(_udpClientThreadFunc, this);
	LOG(DEBUG) << "YawVRSimulatorUdpClient::init: thread created";
	_connectionState = ConnectionState::Disconnected;
	_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time();
	_lastLogTime = boost::posix_time::microsec_clock::universal_time();
}

void YawVRSimulatorClient::shutdown() {
	LOG(TRACE) << "YawVRSimulatorUdpClient::shutdown";
	if (_udpClientThreadRunning) {
		_udpClientThreadStopFlag = true;
		LOG(DEBUG) << "YawVRSimulatorUdpClient::shutdown: waiting end of thread (joining)";
		_udpClientThread.join();
	}
}
void YawVRSimulatorClient::setYawVRSimulatorIPAddress(const std::string& ipAddress) {
	LOG(TRACE) << "YawVRSimulatorUdpClient::setYawVRSimulatorIPAddress( ipAddress : " << ipAddress << " )";
	this->_ipAddress = ipAddress;
	_connectionState = (ConnectionState)((uint8_t)_connectionState | (uint8_t)ConnectionState::Dirty);
}

YawVRSimulatorPacket_t YawVRSimulatorClient::getLastPacket() {
	std::lock_guard<std::mutex> guard(_mutex);
	return m_lastPacket;
}

vr::HmdQuaternion_t YawVRSimulatorClient::getSimRotation() {
	std::lock_guard<std::mutex> guard(_mutex);
	return vrmath::quaternionFromYawPitchRoll(-m_lastPacket.simYaw * M_PI / 180.0, m_lastPacket.simRoll * M_PI / 180.0, m_lastPacket.simPitch * M_PI / 180.0);
}

bool YawVRSimulatorClient::parsePacket(const char *buffer, YawVRSimulatorPacket_t& yawVRSimulatorPacket) {
	boost::regex packetRegEx(YAWVRSIM_UDP_PACKET_REGEX);
	boost::smatch match;
	std::string bufferStr(buffer);
	if (boost::regex_search(bufferStr, match, packetRegEx)) {
		yawVRSimulatorPacket.simYaw = match[YAWVRSIM_UDP_PACKET_SIMYAW].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_SIMYAW]);
		yawVRSimulatorPacket.simPitch = match[YAWVRSIM_UDP_PACKET_SIMPITCH].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_SIMPITCH]);
		yawVRSimulatorPacket.simRoll = match[YAWVRSIM_UDP_PACKET_SIMROLL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_SIMROLL]);
		yawVRSimulatorPacket.gameYaw = match[YAWVRSIM_UDP_PACKET_GAMEYAW].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_GAMEYAW]);
		yawVRSimulatorPacket.gamePitch = match[YAWVRSIM_UDP_PACKET_GAMEPITCH].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_GAMEPITCH]);
		yawVRSimulatorPacket.gameRoll = match[YAWVRSIM_UDP_PACKET_GAMEROLL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_GAMEROLL]);
		yawVRSimulatorPacket.motorSpeed1 = match[YAWVRSIM_UDP_PACKET_MOTORSPD1].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_MOTORSPD1]);
		yawVRSimulatorPacket.motorSpeed2 = match[YAWVRSIM_UDP_PACKET_MOTORSPD2].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_MOTORSPD2]);
		yawVRSimulatorPacket.motorSpeed3 = match[YAWVRSIM_UDP_PACKET_MOTORSPD3].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_MOTORSPD3]);
		yawVRSimulatorPacket.batteryLevel = match[YAWVRSIM_UDP_PACKET_BATTERYLVL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWVRSIM_UDP_PACKET_BATTERYLVL]);
		return true;
	}
	return false;
}

void YawVRSimulatorClient::_udpClientThreadFunc(YawVRSimulatorClient* _this) {
	SOCKET tcpSockfd = INVALID_SOCKET;
	SOCKET sockfd = INVALID_SOCKET;
	SOCKADDR_IN sockAddr;
	int addrLen = sizeof(sockAddr);
	char rcvBuffer[YAWVRSIM_UDP_PORT];
	int rcvSize;

	_this->_udpClientThreadRunning = true;
	LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: thread started";
	try {
		while (!_this->_udpClientThreadStopFlag) {
			try {
				if (((uint8_t)_this->_connectionState & (uint8_t)ConnectionState::ShouldConnect) && sockfd == INVALID_SOCKET && boost::posix_time::microsec_clock::universal_time() > _this->_nextConnectionAttemptTime) {
					if (tcpSockfd == INVALID_SOCKET) {
						LOG(TRACE) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Connecting to YawVR simulator on " << _this->_ipAddress << ":" << YAWVRSIM_TCP_PORT << " ...";
						tcpSockfd = socket(AF_INET, SOCK_STREAM, 0);
						if (tcpSockfd == INVALID_SOCKET) {
							int err = WSAGetLastError();
							throw std::runtime_error(boost::str(boost::format("Could not connect to YawVR simulator : Error code %d.") % err));
						}
						LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: TCP socket created, connecting ...";
						sockAddr.sin_family = AF_INET;
						sockAddr.sin_port = htons(YAWVRSIM_TCP_PORT);
						sockAddr.sin_addr.s_addr = inet_addr(_this->_ipAddress.c_str());
//						InetPton(AF_INET, (PCWSTR)(_this->_ipAddress.c_str()), &sockAddr.sin_addr.s_addr);
						if (::connect(tcpSockfd, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) != 0) {
							int err = WSAGetLastError();
							throw std::runtime_error(boost::str(boost::format("Could not connect to YawVR simulator : Error code %d.") % err));
						}
						LOG(TRACE) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Requesting checkin on UDP port : " << YAWVRSIM_UDP_PORT << " ...";
						char yawVRSimulatorUDPPortIntStr[sizeof(__int32)];
						//*((__int32*)yawVRSimulatorUDPPortIntStr) = (__int32)YAWVRSIM_UDP_PORT;
						__int32 yawVRSimulatorUDPPort = YAWVRSIM_UDP_PORT;
						BYTE *yawVRSimulatorUDPPortBytesArray = (BYTE*)&yawVRSimulatorUDPPort;
						std::reverse_copy(yawVRSimulatorUDPPortBytesArray, yawVRSimulatorUDPPortBytesArray + sizeof yawVRSimulatorUDPPort, yawVRSimulatorUDPPortIntStr);
						std::string yawVRSimulatorUDPPortCheckinTCPPacketRequest;
						yawVRSimulatorUDPPortCheckinTCPPacketRequest.resize(1 + sizeof(__int32) + std::string(YAWVRSIM_UDP_PORT_CHECKIN_GAME_NAME).length() + 1, 0);
						yawVRSimulatorUDPPortCheckinTCPPacketRequest[0] = 0x30;
						yawVRSimulatorUDPPortCheckinTCPPacketRequest.replace(yawVRSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1, yawVRSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1 + sizeof(__int32), yawVRSimulatorUDPPortIntStr, sizeof(__int32));
						yawVRSimulatorUDPPortCheckinTCPPacketRequest.replace(yawVRSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1 + sizeof(__int32), yawVRSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1 + sizeof(__int32) + std::string(YAWVRSIM_UDP_PORT_CHECKIN_GAME_NAME).length(), std::string(YAWVRSIM_UDP_PORT_CHECKIN_GAME_NAME));
						send(tcpSockfd, yawVRSimulatorUDPPortCheckinTCPPacketRequest.data(), (int)yawVRSimulatorUDPPortCheckinTCPPacketRequest.length() + 1, 0);
						if (tcpSockfd == INVALID_SOCKET) {
							int err = WSAGetLastError();
							if (err != WSAETIMEDOUT)
								throw std::runtime_error(boost::str(boost::format("Could not send checkin request to YawVR simulator: Error code %d.") % err));
						}
						rcvSize = recv(tcpSockfd, rcvBuffer, sizeof(rcvBuffer) - 1, 0);
						if (rcvSize != SOCKET_ERROR) {
							rcvBuffer[rcvSize] = '\0';
							LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: received packet : " << rcvBuffer;
						}
						else {
							int err = WSAGetLastError();
							if (err != WSAETIMEDOUT)
								throw std::runtime_error(boost::str(boost::format("Could not receive from YawVR simulator: Error code %d.") % err));
						}
					}

					LOG(TRACE) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Receiving YawVR simulator on 0.0.0.0:" << YAWVRSIM_UDP_PORT << " ...";
					sockfd = socket(AF_INET, SOCK_DGRAM, 0);
					if (sockfd == INVALID_SOCKET) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not create socket to received from YawVR simulator : Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: UDP socket created.";
					struct timeval readTimeout;
					readTimeout.tv_sec = 100;
					readTimeout.tv_usec = 0;
					if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&readTimeout), sizeof readTimeout) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not set timeout socket option: Error code %d.") % err));
					}
					LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Timeout socket option set.";
					sockAddr.sin_family = AF_INET;
					sockAddr.sin_port = htons(YAWVRSIM_UDP_PORT);
					sockAddr.sin_addr.s_addr = INADDR_ANY;
					if (bind(sockfd, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not bind to YawVR simulator : Error code %d.") % err));
					}
					_this->_connectionState = (ConnectionState)(((uint8_t)_this->_connectionState & ~(uint8_t)ConnectionState::Dirty) |
						(uint8_t)(sockfd != INVALID_SOCKET ? ConnectionState::Connected : ConnectionState::Disconnected));
					LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Socket bound.";
				}
				if (sockfd != INVALID_SOCKET) {
					rcvSize = recvfrom(sockfd, rcvBuffer, sizeof(rcvBuffer) - 1, 0, (SOCKADDR*)&sockAddr, &addrLen);
					if (rcvSize != SOCKET_ERROR) {
						rcvBuffer[rcvSize] = '\0';
						try {
							std::lock_guard<std::mutex> guard(_this->_mutex);
							if (parsePacket(rcvBuffer, _this->m_lastPacket)) {
								if (boost::posix_time::microsec_clock::universal_time() > _this->_lastLogTime + boost::posix_time::milliseconds(1000)) {
									_this->_lastLogTime = boost::posix_time::microsec_clock::universal_time();
									LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: received packet : " << _this->m_lastPacket.getString();
								}
							}
							else {
								LOG(ERROR) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: could not parse received packet : " << rcvBuffer;
							}
						} catch (std::exception& ex) {
							LOG(ERROR) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: could not parse received packet : " << rcvBuffer << "\n" << ex.what();
						}
					}
					else {
						int err = WSAGetLastError();
						if (err != WSAETIMEDOUT)
							throw std::runtime_error(boost::str(boost::format("Could not receive from YawVR simulator: Error code %d.") % err));
					}
				}
			} catch (std::exception& ex) {
				_this->_connectionState = (ConnectionState)((uint8_t)_this->_connectionState & ~(uint8_t)ConnectionState::Connected);
				LOG(ERROR) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Exception caught in YawVR simulator UDP client receive loop !\n" << ex.what();
				if ((uint8_t)_this->_connectionState & (uint8_t)ConnectionState::ShouldConnect) {
					_this->_connectionState = (ConnectionState)((uint8_t)_this->_connectionState | (uint8_t)ConnectionState::Dirty);
					LOG(TRACE) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: New attempt in " << YAWVRSIM_CNX_ATTEMPT_DELAY_SEC << " seconds ...";
					_this->_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds(YAWVRSIM_CNX_ATTEMPT_DELAY_SEC * 1000);
				}
			}
			if ((!((uint8_t)_this->_connectionState & (uint8_t)ConnectionState::ShouldConnect) && ((uint8_t)_this->_connectionState & (uint8_t)ConnectionState::Dirty))) {
				if (sockfd != INVALID_SOCKET) {
					closesocket(sockfd);
					sockfd = INVALID_SOCKET;
				}
				if (tcpSockfd != INVALID_SOCKET) {
					LOG(TRACE) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Disconnecting from YawVR simulator";
					closesocket(tcpSockfd);
					tcpSockfd = INVALID_SOCKET;
				}
				_this->_connectionState = (ConnectionState)((uint8_t)_this->_connectionState & ~((uint8_t)ConnectionState::Dirty | (uint8_t)ConnectionState::Connected));
			}
			if (sockfd == INVALID_SOCKET) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			}
		}
		if (sockfd != INVALID_SOCKET) {
			closesocket(sockfd);
			sockfd = INVALID_SOCKET;
		}
		if (tcpSockfd != INVALID_SOCKET) {
			closesocket(tcpSockfd);
			tcpSockfd = INVALID_SOCKET;
		}
	} catch (std::exception& ex) {
		LOG(ERROR) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: Exception caught in YawVR simulator UDP client thread: " << ex.what();
	}
	_this->_udpClientThreadRunning = false;
	LOG(DEBUG) << "YawVRSimulatorUdpClient::_udpClientThreadFunc: thread stopped";
}


} // end namespace driver
} // end namespace vrinputemulator
#endif
