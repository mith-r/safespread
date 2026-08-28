import {
  isPoseUsable,
  MappingStatus,
  PoseUpdatePayload,
  TrackingReason,
} from '../modules/arkit-pose/src/ArkitPose.types';

describe('ARKit tracking eligibility', () => {
  it('allows only normal tracking to drive', () => {
    expect(isPoseUsable('normal')).toBe(true);
    expect(isPoseUsable('limited')).toBe(false);
    expect(isPoseUsable('notAvailable')).toBe(false);
  });

  it('represents every ARKit limited reason', () => {
    const reasons: TrackingReason[] = [
      'initializing',
      'excessiveMotion',
      'insufficientFeatures',
      'relocalizing',
      'unknown',
    ];
    expect(reasons).toHaveLength(5);
  });

  it('represents every world-mapping status', () => {
    const statuses: MappingStatus[] = ['notAvailable', 'limited', 'extending', 'mapped'];
    expect(statuses).toHaveLength(4);
  });

  it('permits an ineligible status event without inventing a pose', () => {
    const event: PoseUpdatePayload = {
      kind: 'status',
      frameTimestampMs: 1250,
      emittedTimestampMs: 1250,
      frameIntervalMs: 0,
      thermalState: 'nominal',
      sequence: 42,
      trackingState: 'notAvailable',
      trackingReason: 'interrupted',
      mappingStatus: 'notAvailable',
    };
    expect('x' in event).toBe(false);
    expect(isPoseUsable(event.trackingState)).toBe(false);
  });
});
