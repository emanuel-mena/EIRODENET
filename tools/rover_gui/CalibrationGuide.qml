import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils

Rectangle {
    id: root
    color: "#111827"
    property url modelUrl: roverModelUrl
    property int faceIndex: 0
    property vector3d targetRotation: {
        switch (faceIndex) {
        case 0: return Qt.vector3d(0, 0, 90)    // +X del rover hacia arriba
        case 1: return Qt.vector3d(0, 0, -90)   // -X del rover hacia arriba
        case 2: return Qt.vector3d(90, 0, 0)    // +Y rover = -Z del modelo
        case 3: return Qt.vector3d(-90, 0, 0)   // -Y rover = +Z del modelo
        case 4: return Qt.vector3d(0, 0, 0)     // +Z: posición normal
        default: return Qt.vector3d(0, 0, 180)  // -Z: rover invertido
        }
    }
    property var instructions: [
        "Gire el rover sobre su costado hasta dejar +X (rojo) hacia arriba.",
        "Gire al costado contrario hasta dejar -X (sentido opuesto al rojo) hacia arriba.",
        "Levante el frente/parte posterior hasta dejar +Y (verde) hacia arriba.",
        "Invierta ese apoyo hasta dejar -Y (sentido opuesto al verde) hacia arriba.",
        "Apoye el rover normalmente: +Z (azul) debe mirar hacia arriba.",
        "Coloque el rover boca abajo: -Z debe mirar hacia arriba. Proteja los componentes."
    ]
    property int assetStatus: rover.status
    property string assetError: rover.errorString

    function restartAnimation() {
        poseAnimation.restart()
    }
    onFaceIndexChanged: restartAnimation()

    View3D {
        anchors.fill: parent
        anchors.topMargin: 92
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#111827"
            antialiasingMode: SceneEnvironment.MSAA
        }
        PerspectiveCamera { z: 17; y: 2; eulerRotation.x: -7 }
        DirectionalLight { eulerRotation.x: -35; eulerRotation.y: -25; brightness: 1.25 }
        DirectionalLight { eulerRotation.x: 30; eulerRotation.y: 150; brightness: 0.55 }

        Node {
            id: pose

            RuntimeLoader {
                id: rover
                source: root.modelUrl
                property real fit: 1.0
                scale: Qt.vector3d(fit, fit, fit)
                onBoundsChanged: {
                    const dx = bounds.maximum.x - bounds.minimum.x
                    const dy = bounds.maximum.y - bounds.minimum.y
                    const dz = bounds.maximum.z - bounds.minimum.z
                    fit = 7.5 / Math.max(dx, dy, dz, 0.001)
                    position = Qt.vector3d(
                        -(bounds.minimum.x + bounds.maximum.x) * 0.5 * fit,
                        -(bounds.minimum.y + bounds.maximum.y) * 0.5 * fit,
                        -(bounds.minimum.z + bounds.maximum.z) * 0.5 * fit)
                }
            }

            // Tríada lógica del rover transformada a los ejes del modelo:
            // X rover -> X modelo, Y rover -> -Z modelo, Z rover -> Y modelo.
            Node {
                id: axes
                property real inactiveOpacity: 0.22

                Model {
                    source: "#Cylinder"; position.x: 1.4; eulerRotation.z: -90
                    scale: Qt.vector3d(0.003, 0.028, 0.003)
                    materials: PrincipledMaterial { baseColor: "#ef4444"; opacity: root.faceIndex < 2 ? 1 : axes.inactiveOpacity }
                }
                Model {
                    source: "#Cone"; position.x: 3.15; eulerRotation.z: -90
                    scale: Qt.vector3d(0.007, 0.014, 0.007)
                    materials: PrincipledMaterial { baseColor: "#ef4444"; opacity: root.faceIndex < 2 ? 1 : axes.inactiveOpacity }
                }
                Model {
                    source: "#Cylinder"; position.z: -1.4; eulerRotation.x: -90
                    scale: Qt.vector3d(0.003, 0.028, 0.003)
                    materials: PrincipledMaterial { baseColor: "#22c55e"; opacity: root.faceIndex >= 2 && root.faceIndex < 4 ? 1 : axes.inactiveOpacity }
                }
                Model {
                    source: "#Cone"; position.z: -3.15; eulerRotation.x: -90
                    scale: Qt.vector3d(0.007, 0.014, 0.007)
                    materials: PrincipledMaterial { baseColor: "#22c55e"; opacity: root.faceIndex >= 2 && root.faceIndex < 4 ? 1 : axes.inactiveOpacity }
                }
                Model {
                    source: "#Cylinder"; position.y: 1.4
                    scale: Qt.vector3d(0.003, 0.028, 0.003)
                    materials: PrincipledMaterial { baseColor: "#3b82f6"; opacity: root.faceIndex >= 4 ? 1 : axes.inactiveOpacity }
                }
                Model {
                    source: "#Cone"; position.y: 3.15
                    scale: Qt.vector3d(0.007, 0.014, 0.007)
                    materials: PrincipledMaterial { baseColor: "#3b82f6"; opacity: root.faceIndex >= 4 ? 1 : axes.inactiveOpacity }
                }
            }

            SequentialAnimation {
                id: poseAnimation
                running: true
                loops: Animation.Infinite
                PauseAnimation { duration: 650 }
                Vector3dAnimation {
                    target: pose; property: "eulerRotation"
                    from: Qt.vector3d(0, 0, 0); to: root.targetRotation
                    duration: 1300; easing.type: Easing.InOutCubic
                }
                PauseAnimation { duration: 1700 }
                Vector3dAnimation {
                    target: pose; property: "eulerRotation"
                    to: Qt.vector3d(0, 0, 0)
                    duration: 900; easing.type: Easing.InOutCubic
                }
                PauseAnimation { duration: 500 }
            }
        }
    }

    Column {
        anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
        anchors.margins: 12; spacing: 5
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: "↑ ARRIBA: el eje seleccionado debe apuntar en esta dirección ↑"
            color: "#fbbf24"; font.pixelSize: 15; font.bold: true
        }
        Text {
            width: parent.width; horizontalAlignment: Text.AlignHCenter
            text: root.instructions[root.faceIndex]
            color: "#e5e7eb"; font.pixelSize: 13; wrapMode: Text.WordWrap
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: "X rojo   •   Y verde   •   Z azul"
            color: "#94a3b8"; font.pixelSize: 12
        }
    }

    Text {
        anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom
        anchors.bottomMargin: 8
        text: "Espere a que el rover esté completamente quieto antes de capturar."
        color: "#cbd5e1"; font.pixelSize: 12
    }
}
