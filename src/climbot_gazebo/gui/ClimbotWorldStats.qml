/* Copyright 2026 jerry
 * Licensed under the Apache License, Version 2.0.
 * See https://www.apache.org/licenses/LICENSE-2.0
 */
import QtQuick 2.12
import QtQuick.Layouts 1.3

Rectangle {
  width: 132
  height: 32
  Layout.minimumWidth: 132
  Layout.minimumHeight: 32
  color: "#dc24313c"
  radius: 5
  Text {
    anchors.centerIn: parent
    text: "RTF " + ClimbotWorldStats.factor
    color: "white"
    font.pixelSize: 14
  }
}
