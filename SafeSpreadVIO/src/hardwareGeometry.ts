import { MountCalibration } from './poseMath';

/**
 * Fixed rover measurements in feet. Ground-plane offsets use the rear-axle
 * midpoint as the origin, with positive forward and positive right.
 */
export const SAFESPREAD_HARDWARE_GEOMETRY = Object.freeze({
  cameraForwardFt: 21 / 12,
  cameraRightFt: 0,
  cameraHeightAboveAxleFt: 20 / 30.48,
  cameraPitchDeg: 45,
  sprayForwardFt: -2.5 / 12,
  sprayRightFt: 0,
  sprayWidthFt: 17 / 12,
  wheelbaseFt: 13.5 / 12,
  frontTrackFt: 19.5 / 30.48,
  rearTrackFt: 19.5 / 30.48,
  // Full-lock turning radii measured with turn_radius/turn_radius.ino. They
  // are not the same size because the steering trim sits off-centre. These
  // mirror the firmware defaults (auto_vio.ino); the rover replaces them with
  // its stored steering calibration when it plans, and reports what it needs.
  turnRadiusLeftFt: 4.33,
  turnRadiusRightFt: 2.92,
});

export const DEFAULT_MOUNT_CALIBRATION: MountCalibration = {
  id: 0,
  schemaVersion: 1,
  cameraForwardFt: SAFESPREAD_HARDWARE_GEOMETRY.cameraForwardFt,
  cameraRightFt: SAFESPREAD_HARDWARE_GEOMETRY.cameraRightFt,
  cameraYawDeg: 0,
  sprayForwardFt: SAFESPREAD_HARDWARE_GEOMETRY.sprayForwardFt,
  sprayRightFt: SAFESPREAD_HARDWARE_GEOMETRY.sprayRightFt,
};
