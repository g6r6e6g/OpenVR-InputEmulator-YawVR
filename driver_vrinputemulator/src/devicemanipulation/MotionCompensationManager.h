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
#ifdef YAWVR
	bool isYawVR3dofModeEnabled() const { return _yawVR3dofModeEnabled; }
	void enableYawVR3dofMode(bool enable);
	bool isYawVRBasedMotionCompensationEnabled() const { return _yawVRBasedMotionCompensationEnabled; }
	void enableYawVRBasedMotionCompensation(bool enable);
	const vr::HmdQuaternion_t& yawVRShellPivotFromCalibrationDeviceRotationOffset() const { return _yawVRShellPivotFromCalibrationDeviceRotationOffset; }
	vr::HmdQuaternion_t& yawVRShellPivotFromCalibrationDeviceRotationOffset() { return _yawVRShellPivotFromCalibrationDeviceRotationOffset; }
	const vr::HmdVector3d_t& yawVRShellPivotFromCalibrationDeviceTranslationOffset() const { return _yawVRShellPivotFromCalibrationDeviceTranslationOffset; }
	vr::HmdVector3d_t& yawVRShellPivotFromCalibrationDeviceTranslationOffset() { return _yawVRShellPivotFromCalibrationDeviceTranslationOffset; }
#endif
	void _disableMotionCompensationOnAllDevices();
	bool _isMotionCompensationZeroPoseValid();
#ifdef YAWVR
	void _setMotionCompensationZeroPose(const vr::DriverPose_t& pose, const vr::HmdQuaternion_t& yawVRSimulatorRotation/*, DeviceManipulationHandle* deviceInfo*/);
#else
	void _setMotionCompensationZeroPose(const vr::DriverPose_t& pose);
#endif
	void _updateMotionCompensationRefPose(const vr::DriverPose_t& pose);
#ifdef YAWVR
	bool _applyMotionCompensation(vr::DriverPose_t& pose, const vr::HmdQuaternion_t& yawVRSimulatorRotation, DeviceManipulationHandle* deviceInfo);
#else
	bool _applyMotionCompensation(vr::DriverPose_t& pose, DeviceManipulationHandle* deviceInfo);
#endif

	void runFrame();

private:
	ServerDriver* m_parent;

	bool _motionCompensationEnabled = false;
	DeviceManipulationHandle* _motionCompensationRefDevice = nullptr;
	MotionCompensationStatus _motionCompensationStatus = MotionCompensationStatus::WaitingForZeroRef;
	constexpr static uint32_t _motionCompensationZeroRefTimeoutMax = 20;
	uint32_t _motionCompensationZeroRefTimeout = 0;
	MotionCompensationVelAccMode _motionCompensationVelAccMode = MotionCompensationVelAccMode::Disabled;
	double m_motionCompensationKalmanProcessVariance = 0.1;
	double m_motionCompensationKalmanObservationVariance = 0.1;
	unsigned m_motionCompensationMovingAverageWindow = 3;
#ifdef YAWVR
	bool _yawVRBasedMotionCompensationEnabled = false;
	bool _yawVR3dofModeEnabled = false;
	vr::HmdQuaternion_t _yawVRShellPivotFromCalibrationDeviceRotationOffset = { 1.0, 0.0, 0.0, 0.0 }; // YawVR simulator shell pivot facing what the controller pointing, same up
	vr::HmdVector3d_t _yawVRShellPivotFromCalibrationDeviceTranslationOffset = { 0.0, -0.10, 0.0 }; // YawVR simulator shell pivot should be 10cm below the controller
#endif

	bool _motionCompensationZeroPoseValid = false;
	vr::HmdVector3d_t _motionCompensationZeroPos;
	vr::HmdQuaternion_t _motionCompensationZeroRot;
#ifdef YAWVR
	vr::HmdQuaternion_t _motionCompensationYawVRSimulatorZeroRot;
#endif

	bool _motionCompensationRefPoseValid = false;
	vr::HmdVector3d_t _motionCompensationRefPos;
	vr::HmdQuaternion_t _motionCompensationRotDiff;
	vr::HmdQuaternion_t _motionCompensationRotDiffInv;
#ifdef YAWVR
	vr::HmdQuaternion_t _motionCompensationYawVRSimulatorRotDiff;
	vr::HmdQuaternion_t _motionCompensationYawVRSimulatorRotDiffInv;
#endif

	bool _motionCompensationRefVelAccValid = false;
	vr::HmdVector3d_t _motionCompensationRefPosVel;
	vr::HmdVector3d_t _motionCompensationRefPosAcc;
	vr::HmdVector3d_t _motionCompensationRefRotVel;
	vr::HmdVector3d_t _motionCompensationRefRotAcc;
};

}
}