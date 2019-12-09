#ifdef YAWVR
#include "YawSimulatorUdpClient.h"

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

#if YAWSIM_EMULATOR
#define YAWSIM_TCP_IP				"192.168.1.18" // Emulator, just launch the Unity Emulator app
#else
#define YAWSIM_TCP_IP				"192.168.1.28" // Real Yaw simulator device, start the Android appand start the device
#endif
#define YAWSIM_TCP_PORT				50020
#define YAWSIM_UDP_PORT				28067
#define YAWSIM_UDP_PORT_CHECKIN_GAME_NAME	"YawVRMotionCompensation"
#define YAWSIM_CNX_ATTEMPT_DELAY_SEC	5		
#define YAWSIM_UDP_PACKET_MAXSIZE	256
#define YAWSIM_UDP_PACKET_SIMYAW		"simYaw"
#define YAWSIM_UDP_PACKET_SIMPITCH	"simPitch"
#define YAWSIM_UDP_PACKET_SIMROLL	"simRoll"
#define YAWSIM_UDP_PACKET_GAMEYAW	"gameYaw"
#define YAWSIM_UDP_PACKET_GAMEPITCH	"gamePitch"
#define YAWSIM_UDP_PACKET_GAMEROLL	"gameRoll"
#define YAWSIM_UDP_PACKET_MOTORSPD1	"motorSpd1"
#define YAWSIM_UDP_PACKET_MOTORSPD2	"motorSpd2"
#define YAWSIM_UDP_PACKET_MOTORSPD3	"motorSpd3"
#define YAWSIM_UDP_PACKET_BATTERYLVL	"batteryLvl"
#define YAWSIM_UDP_PACKET_REGEX		"^(?:SY\\[(?<" ## YAWSIM_UDP_PACKET_SIMYAW ## ">[+-]?\\d+[\\.]?\\d*)])?(?:SP\\[(?<" ## YAWSIM_UDP_PACKET_SIMPITCH ## ">[+-]?\\d+[\\.]?\\d*)])?(?:SR\\[(?<" ## YAWSIM_UDP_PACKET_SIMROLL ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GY\\[(?<" ## YAWSIM_UDP_PACKET_GAMEYAW ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GP\\[(?<" ## YAWSIM_UDP_PACKET_GAMEPITCH ## ">[+-]?\\d+[\\.]?\\d*)])?(?:GR\\[(?<" ## YAWSIM_UDP_PACKET_GAMEROLL ## ">[+-]?\\d+[\\.]?\\d*)])?(?:S1\\[(?<" ## YAWSIM_UDP_PACKET_MOTORSPD1 ## ">\\d+[\\.]?\\d*)\\])?(?:S2\\[(?<" ## YAWSIM_UDP_PACKET_MOTORSPD2 ## ">\\d+[\\.]?\\d*)\\])?(?:S3\\[(?<" ## YAWSIM_UDP_PACKET_MOTORSPD3 ## ">\\d+[\\.]?\\d*)\\])?(?:U\\[(?<" ## YAWSIM_UDP_PACKET_BATTERYLVL ## ">\\d+[\\.]?\\d*)\\])?.*$"

namespace vrinputemulator {
namespace driver {

std::string YawSimulatorPacket_t::getString() {
	return boost::str(boost::format("YawSimulatorPacket(simYaw:%.3f, simPitch:%.3f, simRoll:%.3f, gameYaw:%.3f, gamePitch:%.3f, gameRoll:%.3f, motorSpeed1:%.3f, motorSpeed3:%.3f, motorSpeed3:%.3f, batteryLevel:%.3f)") %
		simYaw % simPitch % simRoll % gameYaw % gamePitch % gameRoll % motorSpeed1 % motorSpeed2 % motorSpeed3 % batteryLevel);
}

void YawSimulatorClient::init() {
	LOG(TRACE) << "YawSimulatorUdpClient::init";
	_udpClientThreadStopFlag = false;
	_udpClientThread = std::thread(_udpClientThreadFunc, this);
	LOG(DEBUG) << "YawSimulatorUdpClient::init: thread created";
	_udpClientThreadShouldConnect = false;
	_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time();
	_lastLogTime = boost::posix_time::microsec_clock::universal_time();
}

void YawSimulatorClient::shutdown() {
	LOG(TRACE) << "YawSimulatorUdpClient::shutdown";
	if (_udpClientThreadRunning) {
		_udpClientThreadStopFlag = true;
		LOG(DEBUG) << "YawSimulatorUdpClient::shutdown: waiting end of thread (joining)";
		_udpClientThread.join();
	}
}

void YawSimulatorClient::setYawSimulatorIPAddress(const std::string& ipAddress) {
	this->_ipAddress = ipAddress;
	_udpClientThreadShouldConnect = false;
}

YawSimulatorPacket_t YawSimulatorClient::getLastPacket() {
	std::lock_guard<std::mutex> guard(_mutex);
	return m_lastPacket;
}

vr::HmdQuaternion_t YawSimulatorClient::getSimRotation() {
	std::lock_guard<std::mutex> guard(_mutex);
	return vrmath::quaternionFromYawPitchRoll(-m_lastPacket.simYaw * M_PI / 180.0, m_lastPacket.simRoll * M_PI / 180.0, m_lastPacket.simPitch * M_PI / 180.0);
}

bool YawSimulatorClient::parsePacket(const char *buffer, YawSimulatorPacket_t& yawSimulatorPacket) {
	boost::regex packetRegEx(YAWSIM_UDP_PACKET_REGEX);
	boost::smatch match;
	std::string bufferStr(buffer);
	if (boost::regex_search(bufferStr, match, packetRegEx)) {
		yawSimulatorPacket.simYaw = match[YAWSIM_UDP_PACKET_SIMYAW].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_SIMYAW]);
		yawSimulatorPacket.simPitch = match[YAWSIM_UDP_PACKET_SIMPITCH].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_SIMPITCH]);
		yawSimulatorPacket.simRoll = match[YAWSIM_UDP_PACKET_SIMROLL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_SIMROLL]);
		yawSimulatorPacket.gameYaw = match[YAWSIM_UDP_PACKET_GAMEYAW].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_GAMEYAW]);
		yawSimulatorPacket.gamePitch = match[YAWSIM_UDP_PACKET_GAMEPITCH].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_GAMEPITCH]);
		yawSimulatorPacket.gameRoll = match[YAWSIM_UDP_PACKET_GAMEROLL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_GAMEROLL]);
		yawSimulatorPacket.motorSpeed1 = match[YAWSIM_UDP_PACKET_MOTORSPD1].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_MOTORSPD1]);
		yawSimulatorPacket.motorSpeed2 = match[YAWSIM_UDP_PACKET_MOTORSPD2].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_MOTORSPD2]);
		yawSimulatorPacket.motorSpeed3 = match[YAWSIM_UDP_PACKET_MOTORSPD3].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_MOTORSPD3]);
		yawSimulatorPacket.batteryLevel = match[YAWSIM_UDP_PACKET_BATTERYLVL].str().empty() ? 0.0 : boost::lexical_cast<float>(match[YAWSIM_UDP_PACKET_BATTERYLVL]);
		return true;
	}
	return false;
}

void YawSimulatorClient::_udpClientThreadFunc(YawSimulatorClient* _this) {
	SOCKET tcpSockfd = INVALID_SOCKET;
	SOCKET sockfd = INVALID_SOCKET;
	SOCKADDR_IN sockAddr;
	int addrLen = sizeof(sockAddr);
	char rcvBuffer[YAWSIM_UDP_PORT];
	int rcvSize;

	_this->_udpClientThreadRunning = true;
	LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: thread started";
	try {
		while (!_this->_udpClientThreadStopFlag) {
			try {
				if (_this->_udpClientThreadShouldConnect && sockfd == INVALID_SOCKET && boost::posix_time::microsec_clock::universal_time() > _this->_nextConnectionAttemptTime) {
					if (tcpSockfd == INVALID_SOCKET) {
						LOG(TRACE) << "YawSimulatorUdpClient::_udpClientThreadFunc: Connecting to Yaw simulator on " << YAWSIM_TCP_IP << ":" << YAWSIM_TCP_PORT << " ...";
						tcpSockfd = socket(AF_INET, SOCK_STREAM, 0);
						if (tcpSockfd == INVALID_SOCKET) {
							int err = WSAGetLastError();
							throw std::runtime_error(boost::str(boost::format("Could not connect to Yaw simulator : Error code %d.") % err));
						}
						LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: TCP socket created, connecting ...";
						sockAddr.sin_family = AF_INET;
						sockAddr.sin_port = htons(YAWSIM_TCP_PORT);
						sockAddr.sin_addr.s_addr = inet_addr(YAWSIM_TCP_IP);
//						InetPton(AF_INET, (PCWSTR)(YAWSIM_TCP_IP), &sockAddr.sin_addr.s_addr);
						if (::connect(tcpSockfd, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) != 0) {
							int err = WSAGetLastError();
							throw std::runtime_error(boost::str(boost::format("Could not connect to Yaw simulator : Error code %d.") % err));
						}
						LOG(TRACE) << "YawSimulatorUdpClient::_udpClientThreadFunc: Requesting checkin on UDP port : " << YAWSIM_UDP_PORT << " ...";
						char yawSimulatorUDPPortIntStr[sizeof(__int32)];
						//*((__int32*)yawSimulatorUDPPortIntStr) = (__int32)YAWSIM_UDP_PORT;
						__int32 yawSimulatorUDPPort = YAWSIM_UDP_PORT;
						BYTE *yawSimulatorUDPPortBytesArray = (BYTE*)&yawSimulatorUDPPort;
						std::reverse_copy(yawSimulatorUDPPortBytesArray, yawSimulatorUDPPortBytesArray + sizeof yawSimulatorUDPPort, yawSimulatorUDPPortIntStr);
						std::string yawSimulatorUDPPortCheckinTCPPacketRequest;
						yawSimulatorUDPPortCheckinTCPPacketRequest.resize(1 + sizeof(__int32) + std::string(YAWSIM_UDP_PORT_CHECKIN_GAME_NAME).length() + 1, 0);
						yawSimulatorUDPPortCheckinTCPPacketRequest[0] = 0x30;
						yawSimulatorUDPPortCheckinTCPPacketRequest.replace(yawSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1, yawSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1 + sizeof(__int32), yawSimulatorUDPPortIntStr, sizeof(__int32));
						yawSimulatorUDPPortCheckinTCPPacketRequest.replace(yawSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1 + sizeof(__int32), yawSimulatorUDPPortCheckinTCPPacketRequest.begin() + 1 + sizeof(__int32) + std::string(YAWSIM_UDP_PORT_CHECKIN_GAME_NAME).length(), std::string(YAWSIM_UDP_PORT_CHECKIN_GAME_NAME));
						send(tcpSockfd, yawSimulatorUDPPortCheckinTCPPacketRequest.data(), (int)yawSimulatorUDPPortCheckinTCPPacketRequest.length() + 1, 0);
						if (tcpSockfd == INVALID_SOCKET) {
							int err = WSAGetLastError();
							if (err != WSAETIMEDOUT)
								throw std::runtime_error(boost::str(boost::format("Could not send checkin request to Yaw simulator: Error code %d.") % err));
						}
						rcvSize = recv(tcpSockfd, rcvBuffer, sizeof(rcvBuffer) - 1, 0);
						if (rcvSize != SOCKET_ERROR) {
							rcvBuffer[rcvSize] = '\0';
							LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: received packet : " << rcvBuffer;
						}
						else {
							int err = WSAGetLastError();
							if (err != WSAETIMEDOUT)
								throw std::runtime_error(boost::str(boost::format("Could not receive from Yaw simulator: Error code %d.") % err));
						}
					}

					LOG(TRACE) << "YawSimulatorUdpClient::_udpClientThreadFunc: Receiving Yaw simulator on 0.0.0.0:" << YAWSIM_UDP_PORT << " ...";
					sockfd = socket(AF_INET, SOCK_DGRAM, 0);
					if (sockfd == INVALID_SOCKET) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not create socket to received from Yaw simulator : Error code %d.") % err));
					}
					LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: UDP socket created.";
					struct timeval readTimeout;
					readTimeout.tv_sec = 100;
					readTimeout.tv_usec = 0;
					if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&readTimeout), sizeof readTimeout) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not set timeout socket option: Error code %d.") % err));
					}
					LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: Timeout socket option set.";
					sockAddr.sin_family = AF_INET;
					sockAddr.sin_port = htons(YAWSIM_UDP_PORT);
					sockAddr.sin_addr.s_addr = INADDR_ANY;
					if (bind(sockfd, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) != 0) {
						int err = WSAGetLastError();
						throw std::runtime_error(boost::str(boost::format("Could not bind to Yaw simulator : Error code %d.") % err));
					}
					_this->_udpClientThreadConnected = (sockfd != INVALID_SOCKET);
					LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: Socket bound.";
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
									LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: received packet : " << _this->m_lastPacket.getString();
								}
							}
							else {
								LOG(ERROR) << "YawSimulatorUdpClient::_udpClientThreadFunc: could not parse received packet : " << rcvBuffer;
							}
						} catch (std::exception& ex) {
							LOG(ERROR) << "YawSimulatorUdpClient::_udpClientThreadFunc: could not parse received packet : " << rcvBuffer << "\n" << ex.what();
						}
					}
					else {
						int err = WSAGetLastError();
						if (err != WSAETIMEDOUT)
							throw std::runtime_error(boost::str(boost::format("Could not receive from Yaw simulator: Error code %d.") % err));
					}
				}
			} catch (std::exception& ex) {
				_this->_udpClientThreadConnected = false;
				LOG(ERROR) << "YawSimulatorUdpClient::_udpClientThreadFunc: Exception caught in Yaw simulator UDP client receive loop !\n" << ex.what() << "\nNew attempt in " << YAWSIM_CNX_ATTEMPT_DELAY_SEC << " seconds ...";
				_this->_nextConnectionAttemptTime = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds(YAWSIM_CNX_ATTEMPT_DELAY_SEC*1000);
			}
			if (!_this->_udpClientThreadShouldConnect || _this->_nextConnectionAttemptTime > boost::posix_time::microsec_clock::universal_time()) {
				if (sockfd != INVALID_SOCKET) {
					closesocket(sockfd);
					sockfd = INVALID_SOCKET;
				}
				if (tcpSockfd != INVALID_SOCKET) {
					closesocket(tcpSockfd);
					tcpSockfd = INVALID_SOCKET;
				}
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
		LOG(ERROR) << "YawSimulatorUdpClient::_udpClientThreadFunc: Exception caught in Yaw simulator UDP client thread: " << ex.what();
	}
	_this->_udpClientThreadRunning = false;
	LOG(DEBUG) << "YawSimulatorUdpClient::_udpClientThreadFunc: thread stopped";
}


} // end namespace driver
} // end namespace vrinputemulator
#endif
