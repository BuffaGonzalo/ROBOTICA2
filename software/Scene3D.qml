import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import "resources"

View3D {
    id: root

    // ── Propiedades que C++ escribe con setProperty() ──────────────
    // Los valores que llegan desde el MPU ya son grados (ej: -2.14, 1.85)
    property real carPitch: 0   // cabeceo  → eje X del mundo
    property real carRoll:  0   // balanceo → eje Z del mundo
    property real carYaw:   0   // giro     → eje Y del mundo
    property bool isConnected: false

    // ── Rotación de exhibición cuando está desconectado ────────────
    property real idleYaw: 0

    readonly property real targetPitch: root.isConnected ? root.carPitch : 0
    readonly property real targetRoll:  root.isConnected ? root.carRoll  : 0
    readonly property real targetYaw:   root.isConnected ? root.carYaw   : root.idleYaw

    property real activePitch: targetPitch
    property real activeRoll:  targetRoll

    // Amortiguación suave a 0° solo al desconectar (latencia 0 durante telemetría activa)
    Behavior on activePitch {
        enabled: !root.isConnected
        NumberAnimation { duration: 600; easing.type: Easing.OutCubic }
    }

    Behavior on activeRoll {
        enabled: !root.isConnected
        NumberAnimation { duration: 600; easing.type: Easing.OutCubic }
    }

    // Rotación continua lenta y fluida en Yaw (16 segundos por vuelta completa = 22.5°/s)
    NumberAnimation {
        id: idleRotation
        target: root
        property: "idleYaw"
        from: 0
        to: 360
        duration: 16000
        loops: Animation.Infinite
        running: !root.isConnected
    }

    onIsConnectedChanged: {
        if (!isConnected) {
            // Reanudar la rotación continua desde el ángulo actual para evitar saltos bruscos
            var startYaw = (root.carYaw % 360 + 360) % 360
            idleRotation.stop()
            idleRotation.from = startYaw
            idleRotation.to = startYaw + 360
            idleRotation.start()
        } else {
            idleRotation.stop()
        }
    }

    // ── Función que construye el quaternion final ──────────────────
    // Un quaternion representa una rotación como:
    //   q = (cos(ángulo/2), sin(ángulo/2)*eje_x, sin(ángulo/2)*eje_y, sin(ángulo/2)*eje_z)
    // Qt.quaternion(w, x, y, z)
    //
    // IMPORTANTE: los ángulos vienen en GRADOS desde el MPU,
    // hay que convertirlos a RADIANES para Math.sin/cos
    // Dividimos el ángulo por 2 porque el quaternion trabaja con la mitad del ángulo

    function buildRotation(pitch, roll, yaw) {

        // Convertimos cada ángulo a radianes y lo dividimos por 2
        var halfP = pitch * Math.PI / 360.0   // pitch/2 en radianes
        var halfR = -roll  * Math.PI / 360.0   // roll/2  en radianes
        var halfY = yaw   * Math.PI / 360.0   // yaw/2   en radianes

        // Quaternion para Pitch (rotación alrededor del eje X)
        // eje X = (1, 0, 0)  →  Qt.quaternion(w, x*sin, y*sin, z*sin)
        var qPitch = Qt.quaternion(
            Math.cos(halfP),   // w
            Math.sin(halfP),   // x ← eje X activo
            0,                 // y
            0                  // z
        )

        // Quaternion para Yaw (rotación alrededor del eje Y)
        // eje Y = (0, 1, 0)
        var qYaw = Qt.quaternion(
            Math.cos(halfY),   // w
            0,                 // x
            Math.sin(halfY),   // y ← eje Y activo
            0                  // z
        )

        // Quaternion para Roll (rotación alrededor del eje Z)
        // eje Z = (0, 0, 1)
        var qRoll = Qt.quaternion(
            Math.cos(halfR),   // w
            0,                 // x
            0,                 // y
            Math.sin(halfR)    // z ← eje Z activo
        )

        // Multiplicamos en orden aeronáutico: Yaw → Pitch → Roll
        // La multiplicación de quaternions combina las rotaciones
        // SIN que una afecte los ejes de la otra (no hay Gimbal Lock)
        return qYaw.times(qPitch).times(qRoll)
    }

    // ── Entorno ────────────────────────────────────────────────────
    environment: SceneEnvironment {
        clearColor: "#b0c4de"
        backgroundMode: SceneEnvironment.Color
    }

    // ── Cámara ─────────────────────────────────────────────────────
    PerspectiveCamera {
        id: camera
        z: 220
        y: 120
        eulerRotation.x: -15
    }

    // ── Control de órbita con el mouse ─────────────────────────────
    OrbitCameraController {
        anchors.fill: parent
        origin: carModel
        camera: camera
    }

    // ── Iluminación ────────────────────────────────────────────────
    DirectionalLight {
        eulerRotation.x: -45
        eulerRotation.y: 45
        ambientColor: Qt.rgba(0.5, 0.5, 0.5, 1.0)
    }

    // ── Suelo (Plano horizontal) ───────────────────────────────────
    Model {
        id: groundPlane
        source: "#Rectangle"

        // Posición vertical: Ajusta este valor (ej. -50 o -100) si las
        // ruedas del auto quedan "enterradas" en el piso.
        y: -30

        // Escala: Como el auto está escalado a 1500, el suelo debe ser masivo
        scale: Qt.vector3d(5000, 5000, 1)

        // Rotación: Acuesta el rectángulo para que actúe como piso
        eulerRotation.x: -90

        materials: PrincipledMaterial {
            baseColor: "#9ca3af" // Gris claro mate suave acorde a la interfaz
            roughness: 0.85      // Opaco, para que no parezca un espejo
            metalness: 0.0
        }
    }

    // ── Modelo 3D del robot ────────────────────────────────────────
    AutoMicroSimplify {
        id: carModel
        position: Qt.vector3d(0, 0, 0)
        scale: Qt.vector3d(1500, 1500, 1500)

        // ✅ Aquí es donde se aplica el quaternion al modelo
        // En modo desconectado: rota continuamente en Yaw a velocidad suave de exhibición
        // En modo conectado: sigue la orientación física real recibida del MPU en tiempo real
        rotation: root.buildRotation(root.activePitch, root.activeRoll, root.targetYaw)
    }
}
