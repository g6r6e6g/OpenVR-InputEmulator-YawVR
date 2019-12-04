#pragma once

#include <openvr_driver.h>
#include <vrinputemulator_types.h>
#include <openvr_math.h>
#include "../logging.h"



// driver namespace
namespace vrinputemulator {
namespace driver {


// forward declarations
class ServerDriver;
class DeviceManipulationHandle;


enum class MotionCompensationStatus : uint32_t {
	WaitingForZeroRef = 0,
	Running = 1,
	MotionRefNotTracking = 2
};


class MotionCompensationManager {
public:
	MotionCompensationManager(ServerDriver* parent) : m_parent(parent) {}

	void enableMotionCompensation(bool enable);
	MotionCompensationStatus motionCompensationStatus() { return _motionCompensationStatus; }
	void _setMotionCompensationStatus(MotionCompensationStatus status) { _motionCompensationStatus = status; }
	void setMotionCompensationRefDevice(DeviceManipulationHandle* device);
	DeviceManipulationHandle* getMotionCompensationRefDevice();
	void setMotionCompensationVelAccMode(MotionCompensationVelAccMode velAccMode);
	double motionCompensationKalmanProcessVariance() { return m_motionCompensationKalmanProcessVariance; }
	void setMotionCompensationKalmanProcessVariance(double variance);
	double motionCompensationKalmanObservationVariance() { return m_motionCompensationKalmanObservationVariance; }
	void setMotionCompensationKalmanObservationVariance(double variance);
	double motionCompensationMovingAverageWindow() { return m_motionCompensationMovingAverageWindow; }
	void setMotionCompensationMovingAverageWindow(unsigned window);
	void _disableMotionCompensationOnAllDevices();
	bool _isMotionCompensationZeroPoseValid();
#ifdef YAWVR
	void _setMotionCompensationZeroPose(const vr::DriverPose_t& pose, const vr::HmdQuaternion_t& yawVRSimRotation, DeviceManipulationHandle* deviceInfo);
	bool _applyMotionCompensation(vr::DriverPose_t& pose, const vr::HmdQuaternion_t& yawVRSimRotation, DeviceManipulationHandle* deviceInfo);
#else
	void _setMotionCompensationZeroPose(const vr::DriverPose_t& pose);
	void _updateMotionCompensationRefPose(const vr::DriverPose_t& pose);
	bool _applyMotionCompensation(vr::DriverPose_t& pose, DeviceManipulationHandle* deviceInfo);
#endif

	void runFrame();

private:
	ServerDriver* m_parent;

#ifdef YAWVR
	static vr::HmdVector3d_t ctrlr2YawVRShellPivot; // controller to shell pivot offset
#endif
	bool _motionCompensationEnabled = false;
	DeviceManipulationHandle* _motionCompensationRefDevice = nullptr;
	MotionCompensationStatus _motionCompensationStatus = MotionCompensationStatus::WaitingForZeroRef;
	constexpr static uint32_t _motionCompensationZeroRefTimeoutMax = 20;
	uint32_t _motionCompensationZeroRefTimeout = 0;
	MotionCompensationVelAccMode _motionCompensationVelAccMode = MotionCompensationVelAccMode::Disabled;
	double m_motionCompensationKalmanProcessVariance = 0.1;
	double m_motionCompensationKalmanObservationVariance = 0.1;
	unsigned m_motionCompensationMovingAverageWindow = 3;

	bool _motionCompensationZeroPoseValid = false;
	vr::HmdVector3d_t _motionCompensationZeroPos;
	vr::HmdQuaternion_t _motionCompensationZeroRot;
#ifdef YAWVR
	vr::HmdQuaternion_t _motionCompensationYawVRZeroRot;
#endif

	bool _motionCompensationRefPoseValid = false;
	vr::HmdVector3d_t _motionCompensationRefPos;
	vr::HmdQuaternion_t _motionCompensationRotDiff;
	vr::HmdQuaternion_t _motionCompensationRotDiffInv;
#ifdef YAWVR
	vr::HmdQuaternion_t _motionCompensationYawVRRotDiff;
	vr::HmdQuaternion_t _motionCompensationYawVRRotDiffInv;
#endif

	bool _motionCompensationRefVelAccValid = false;
	vr::HmdVector3d_t _motionCompensationRefPosVel;
	vr::HmdVector3d_t _motionCompensationRefPosAcc;
	vr::HmdVector3d_t _motionCompensationRefRotVel;
	vr::HmdVector3d_t _motionCompensationRefRotAcc;
};

}
}