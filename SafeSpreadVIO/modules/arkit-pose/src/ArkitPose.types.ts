export type TrackingState = 'normal' | 'limited' | 'notAvailable';
export type TrackingReason =
  | 'none'
  | 'initializing'
  | 'excessiveMotion'
  | 'insufficientFeatures'
  | 'relocalizing'
  | 'interrupted'
  | 'sessionFailed'
  | 'unknown';
export type MappingStatus = 'notAvailable' | 'limited' | 'extending' | 'mapped' | 'unknown';
/** iOS's own view of how hot the phone is. ARKit halves its frame rate under
 *  sustained thermal pressure, and a pose stream that thins out is what the
 *  rover eventually reports as a pose timeout. */
export type ThermalState = 'nominal' | 'fair' | 'serious' | 'critical' | 'unknown';

/**
 * Whether a pose is trustworthy enough to drive the rover from.
 * Autonomous motion requires ARKit's strongest tracking state. A limited pose
 * remains useful for UI diagnostics but is never eligible to drive the rover.
 */
export function isPoseUsable(state: TrackingState): boolean {
  return state === 'normal';
}

type TrackingMetadata = {
  frameTimestampMs: number;
  emittedTimestampMs: number;
  /** Gap to the previous ARKit frame, zero on the first one. */
  frameIntervalMs: number;
  thermalState: ThermalState;
  sequence: number;
  trackingState: TrackingState;
  trackingReason: TrackingReason;
  mappingStatus: MappingStatus;
};

export type PoseUpdatePayload = TrackingMetadata & {
  kind: 'pose';
  x: number;
  y: number;
  heading: number;
} | TrackingMetadata & {
  kind: 'status';
  error?: string;
};
