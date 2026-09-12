import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils

Rectangle {
    id: root
    color: "#111827"
    property url modelUrl: roverModelUrl
    property quaternion roverRotation: Qt.quaternion(1, 0, 0, 0)
    property int assetStatus: rover.status
    property string assetError: rover.errorString

    View3D {
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#111827"
            antialiasingMode: SceneEnvironment.MSAA
        }
        PerspectiveCamera { id: camera; z: 16; y: 2; eulerRotation.x: -7 }
        DirectionalLight { eulerRotation.x: -35; eulerRotation.y: -25; brightness: 1.2 }
        DirectionalLight { eulerRotation.x: 35; eulerRotation.y: 155; brightness: 0.6 }

        Node {
            rotation: root.roverRotation
            RuntimeLoader {
                id: rover
                source: root.modelUrl
                property real fit: 1.0
                scale: Qt.vector3d(fit, fit, fit)
                onBoundsChanged: {
                    const dx = bounds.maximum.x - bounds.minimum.x
                    const dy = bounds.maximum.y - bounds.minimum.y
                    const dz = bounds.maximum.z - bounds.minimum.z
                    fit = 8.0 / Math.max(dx, dy, dz, 0.001)
                    position = Qt.vector3d(
                        -(bounds.minimum.x + bounds.maximum.x) * 0.5 * fit,
                        -(bounds.minimum.y + bounds.maximum.y) * 0.5 * fit,
                        -(bounds.minimum.z + bounds.maximum.z) * 0.5 * fit)
                }
            }
        }
    }
    Text {
        anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 10
        text: "Rover X→modelo X, Rover Y→modelo -Z, Rover Z→modelo Y"
        color: "#94a3b8"; font.pixelSize: 11
    }
}
