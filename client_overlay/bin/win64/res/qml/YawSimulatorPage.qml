import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.2
import matzman666.inputemulator 1.0

MyStackViewPage {
    id: yawSimulatorPage

    headerText: "Yaw Simulator Settings"

    property bool setupFinished: false

    content: ColumnLayout {
        spacing: 18

        RowLayout {
            spacing: 18

            MyText {
                text: "IP address:"
            }
        }



        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }


        Component.onCompleted: {
            setupFinished = true
        }

    }

}
