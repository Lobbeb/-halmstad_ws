import QtQuick 2.9
import QtQuick.Controls 2.5
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

Rectangle {
  implicitWidth: 320
  implicitHeight: 128
  color: "transparent"

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 10

    Label {
      Layout.fillWidth: true
      text: "Next UAV: " + UavSpawner.nextName
      wrapMode: Text.Wrap
    }

    Button {
      Layout.fillWidth: true
      enabled: !UavSpawner.busy
      text: UavSpawner.busy ? "Spawning..." : "Spawn UAV"
      onClicked: UavSpawner.Spawn()
    }

    Label {
      Layout.fillWidth: true
      text: UavSpawner.statusText
      wrapMode: Text.Wrap
      opacity: 0.85
      font.pixelSize: 12
    }
  }
}
