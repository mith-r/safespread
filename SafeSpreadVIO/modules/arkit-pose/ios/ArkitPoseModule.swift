import ExpoModulesCore
import ARKit

public class ArkitPoseModule: Module {
  private var session: ARSession?
  private var delegate: ArkitSessionDelegate?

  public func definition() -> ModuleDefinition {
    Name("ArkitPose")

    Events("onPoseUpdate")

    Function("start") {
      let session = ARSession()
      let delegate = ArkitSessionDelegate { [weak self] payload in
        self?.sendEvent("onPoseUpdate", payload)
      }
      session.delegate = delegate

      let config = ARWorldTrackingConfiguration()
      config.worldAlignment = .gravity
      session.run(config)

      self.session = session
      self.delegate = delegate
    }

    Function("stop") {
      self.session?.pause()
      self.session = nil
      self.delegate = nil
    }
  }
}

private class ArkitSessionDelegate: NSObject, ARSessionDelegate {
  private let onUpdate: ([String: Any]) -> Void
  private var sequence: UInt32 = 0
  private var lastFrameTimestamp: TimeInterval?

  // ARKit stopped delivering frames at 60 Hz partway through the 2026-08-28
  // mission and decayed to 19 Hz by the end, which the rover eventually called
  // a pose timeout. Nothing in the log said why, because nothing was recording
  // the two things that would distinguish a throttled phone from a busy one:
  // how hot iOS thinks it is, and how far apart the frames are actually
  // arriving. Both are cheap and both are now on every pose.
  //
  // Thermal state is cached from its change notification rather than read per
  // frame; it changes a handful of times an hour, not sixty times a second.
  private var thermalState: String = ArkitSessionDelegate.name(
    for: ProcessInfo.processInfo.thermalState)

  init(onUpdate: @escaping ([String: Any]) -> Void) {
    self.onUpdate = onUpdate
    super.init()
    NotificationCenter.default.addObserver(
      self,
      selector: #selector(thermalStateChanged),
      name: ProcessInfo.thermalStateDidChangeNotification,
      object: nil)
  }

  deinit {
    NotificationCenter.default.removeObserver(self)
  }

  @objc private func thermalStateChanged() {
    thermalState = ArkitSessionDelegate.name(for: ProcessInfo.processInfo.thermalState)
  }

  private static func name(for state: ProcessInfo.ThermalState) -> String {
    switch state {
    case .nominal: return "nominal"
    case .fair: return "fair"
    case .serious: return "serious"
    case .critical: return "critical"
    @unknown default: return "unknown"
    }
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    let t = frame.camera.transform
    let metersToFeet = 3.28084

    // Ground-plane convention: y = forward (world -Z), x = right
    // (world +X), heading 0=+Y, 90=+X.
    let xFt = Double(t.columns.3.x) * metersToFeet
    let yFt = Double(-t.columns.3.z) * metersToFeet

    // ARCamera's local +X axis follows the phone's long axis from its top
    // (front-camera end) toward its bottom. Project local -X onto the pavement
    // so heading follows the phone top despite the fixed 45-degree pitch.
    let forwardX = Double(-t.columns.0.x)
    let forwardZ = Double(-t.columns.0.z)
    var headingDeg = atan2(forwardX, -forwardZ) * 180.0 / Double.pi
    if headingDeg < 0 { headingDeg += 360 }

    let trackingState: String
    let trackingReason: String
    switch frame.camera.trackingState {
    case .normal:
      trackingState = "normal"
      trackingReason = "none"
    case .limited(let reason):
      trackingState = "limited"
      switch reason {
      case .initializing: trackingReason = "initializing"
      case .excessiveMotion: trackingReason = "excessiveMotion"
      case .insufficientFeatures: trackingReason = "insufficientFeatures"
      case .relocalizing: trackingReason = "relocalizing"
      @unknown default: trackingReason = "unknown"
      }
    case .notAvailable:
      trackingState = "notAvailable"
      trackingReason = "unknown"
    }

    let mappingStatus: String
    switch frame.worldMappingStatus {
    case .notAvailable: mappingStatus = "notAvailable"
    case .limited: mappingStatus = "limited"
    case .extending: mappingStatus = "extending"
    case .mapped: mappingStatus = "mapped"
    @unknown default: mappingStatus = "unknown"
    }

    let frameIntervalMs = lastFrameTimestamp.map { (frame.timestamp - $0) * 1000.0 } ?? 0.0
    lastFrameTimestamp = frame.timestamp

    sequence &+= 1
    onUpdate([
      "kind": "pose",
      "x": xFt,
      "y": yFt,
      "heading": headingDeg,
      "trackingState": trackingState,
      "trackingReason": trackingReason,
      "mappingStatus": mappingStatus,
      "frameTimestampMs": frame.timestamp * 1000.0,
      "emittedTimestampMs": ProcessInfo.processInfo.systemUptime * 1000.0,
      "frameIntervalMs": frameIntervalMs,
      "thermalState": thermalState,
      "sequence": sequence,
    ])
  }

  func session(_ session: ARSession, didFailWithError error: Error) {
    emitStatus(reason: "sessionFailed", error: error.localizedDescription)
  }

  func sessionWasInterrupted(_ session: ARSession) {
    emitStatus(reason: "interrupted")
  }

  func sessionInterruptionEnded(_ session: ARSession) {
    emitStatus(state: "limited", reason: "relocalizing")
  }

  private func emitStatus(
    state: String = "notAvailable",
    reason: String,
    error: String? = nil
  ) {
    var payload: [String: Any] = [
      "kind": "status",
      "trackingState": state,
      "trackingReason": reason,
      "mappingStatus": "notAvailable",
      "frameTimestampMs": ProcessInfo.processInfo.systemUptime * 1000.0,
      "emittedTimestampMs": ProcessInfo.processInfo.systemUptime * 1000.0,
      "frameIntervalMs": 0.0,
      "thermalState": thermalState,
      "sequence": sequence,
    ]
    if let error { payload["error"] = error }
    onUpdate(payload)
  }
}
