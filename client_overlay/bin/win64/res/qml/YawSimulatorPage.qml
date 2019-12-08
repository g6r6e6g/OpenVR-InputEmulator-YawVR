import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.2
import matzman666.inputemulator 1.0

MyStackViewPage {
    id: yawSimulatorPage

    headerText: "Yaw Simulator Settings"

    property int deviceIndex: -1
    property bool setupFinished: false

    function setDeviceIndex(index) {
        deviceIndex = index
        updateDeviceInfo()
        yawShellPivotFromCalibrationDeviceOffsetBox.updateGUI()
    }

    function updateDeviceInfo() {
        if (deviceIndex >= 0) {
            yawShellPivotFromCalibrationDeviceOffsetBox.updateValues()
            yawBasedMotionCompensationEnableToggle.checked = DeviceManipulationTabController.yawBasedMotionCompensationEnabled(deviceIndex)
        }
    }

    content: ColumnLayout {
        spacing: 18

        MyToggleButton {
            id: yawBasedMotionCompensationEnableToggle
            text: "Enable Yaw based Motion Compensation"
            Layout.fillWidth: false
            onCheckedChanged: {
                DeviceManipulationTabController.enableYawBasedMotionCompensation(deviceIndex, checked)
            }
        }

        MyIPAddressGroupBox {
            boxTitle: "Yaw IP address (refer to your YawVR config application)"
            id: yawSimulatorIPAddressBox
            setIPAddress: function(part1, part2, part3, part4) {
                if (deviceIndex >= 0) {
                    DeviceManipulationTabController.setYawSimulatorIPAddress(deviceIndex, part1, part2, part3, part4)
                }
            }
            updateValues: function() {
                var hasChanged = false
                var value = DeviceManipulationTabController.getYawSimulatorIPAddress(deviceIndex, 0)
                if (part1 != value) {
                    part1 = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawSimulatorIPAddress(deviceIndex, 1)
                if (part2 != value) {
                    part2 = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawSimulatorIPAddress(deviceIndex, 2)
                if (part3 != value) {
                    part3 = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawSimulatorIPAddress(deviceIndex, 3)
                if (part4 != value) {
                    part4 = value
                    hasChanged = true
                }
                if (hasChanged) {
                    updateGUI()
                }
            }
        }

        MyOffsetGroupBox {
            boxTitle: "Shell pivot from calibration device Offset"
            id: yawShellPivotFromCalibrationDeviceOffsetBox
            setTranslationOffset: function(x, y, z) {
                if (deviceIndex >= 0) {
                    DeviceManipulationTabController.setYawShellPivotFromCalibrationDeviceTranslationOffset(deviceIndex, x, y, z)
                }
            }
            setRotationOffset: function(yaw, pitch, roll) {
                if (deviceIndex >= 0) {
                    DeviceManipulationTabController.setYawShellPivotFromCalibrationDeviceRotationOffset(deviceIndex, yaw, pitch, roll)
                }
            }
            updateValues: function() {
                var hasChanged = false
                var value = DeviceManipulationTabController.getYawShellPivotFromCalibrationDeviceRotationOffset(deviceIndex, 0)
                if (offsetYaw != value) {
                    offsetYaw = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawShellPivotFromCalibrationDeviceRotationOffset(deviceIndex, 1)
                if (offsetPitch != value) {
                    offsetPitch = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawShellPivotFromCalibrationDeviceRotationOffset(deviceIndex, 2)
                if (offsetRoll != value) {
                    offsetRoll = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawShellPivotFromCalibrationDeviceTranslationOffset(deviceIndex, 0)
                if (offsetX != value) {
                    offsetX = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawShellPivotFromCalibrationDeviceTranslationOffset(deviceIndex, 1)
                if (offsetY != value) {
                    offsetY = value
                    hasChanged = true
                }
                value = DeviceManipulationTabController.getYawShellPivotFromCalibrationDeviceTranslationOffset(deviceIndex, 2)
                if (offsetZ != value) {
                    offsetZ = value
                    hasChanged = true
                }
                if (hasChanged) {
                    updateGUI()
                }
            }
        }

        RowLayout {
            MyPushButton {
                Layout.preferredWidth: 200
                text: "Clear"
                onClicked: {
                    yawShellPivotFromCalibrationDeviceOffsetBox.setTranslationOffset(0,0,0)
                    yawShellPivotFromCalibrationDeviceOffsetBox.setRotationOffset(0,0,0)
                }
            }
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        Connections {
            target: DeviceManipulationTabController
            onDeviceInfoChanged: {
                if (index == deviceIndex) {
                    updateDeviceInfo()
                }
            }
        }

        Component.onCompleted: {
            setupFinished = true
        }

    }

}
