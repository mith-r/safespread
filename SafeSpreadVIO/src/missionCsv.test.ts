import { MISSION_CSV_COLUMNS, missionJsonlToCsv } from './missionCsv';

/** The expected row, addressed by column name. Writing these out as a bare
 *  string of commas means adding a column becomes an exercise in counting them,
 *  and a miscount looks exactly like a real failure. */
function row(values: Partial<Record<(typeof MISSION_CSV_COLUMNS)[number], string>>): string {
  return MISSION_CSV_COLUMNS.map((column) => values[column] ?? '').join(',');
}

describe('missionJsonlToCsv', () => {
  it('uses stable replay columns and leaves missing optional fields empty', () => {
    const jsonl = [
      JSON.stringify({ type: 'metadata', missionId: 'm1' }),
      JSON.stringify({
        type: 'pose', phoneMs: 10, sequence: 2, epoch: 7,
        xFt: 1.5, yFt: -2, headingDeg: 359, speedFps: 0.8,
        yawRateDps: -1.25, trackingValid: true,
      }),
    ].join('\n');
    const lines = missionJsonlToCsv(jsonl).trimEnd().split('\n');
    expect(lines[0]).toBe(MISSION_CSV_COLUMNS.join(','));
    expect(lines[1]).toBe(row({
      phone_ms: '10', sequence: '2', epoch: '7', x_ft: '1.5', y_ft: '-2',
      heading_deg: '359', speed_fps: '0.8', yaw_rate_dps: '-1.25',
      tracking_valid: 'true',
    }));
  });

  it('quotes commas, quotes, and newlines', () => {
    const csv = missionJsonlToCsv(JSON.stringify({
      type: 'fault', phoneMs: 2, fault: 'bad, "sensor"\nretry',
    }));
    expect(csv).toContain('"bad, ""sensor""\nretry"');
  });

  it('exports a firmware fault-buffer code through the replay fault column', () => {
    const csv = missionJsonlToCsv(JSON.stringify({
      type: 'fault_buffer', phoneMs: 100, sequence: 2, epoch: 42,
      speedFps: 1.1, routeIndex: 7, crossTrackFt: 0.2,
      headingErrorDeg: 1.5, steeringUs: 1700, throttleUs: 1600,
      faultCode: 9,
    }));
    expect(csv.trimEnd().split('\n')[1]).toBe(row({
      phone_ms: '100', sequence: '2', epoch: '42', speed_fps: '1.1',
      route_index: '7', cross_track_ft: '0.2', heading_error_deg: '1.5',
      steering_us: '1700', throttle_us: '1600', fault: '9',
    }));
  });

  // Why the pose stream thinned out, and whether the frame it was reported in
  // is the frame it belongs to. Working either out from the raw JSONL is what
  // the 2026-08-28 investigation had to do by hand.
  it('exports the pose-timing and frame-shift diagnostics', () => {
    const csv = missionJsonlToCsv(JSON.stringify({
      type: 'pose', phoneMs: 5, captureAgeMs: 161, frameIntervalMs: 52.4,
      thermalState: 'serious', poseAgeMs: 252, relocalizationShiftFt: 1.11,
    }));
    expect(csv.trimEnd().split('\n')[1]).toBe(row({
      phone_ms: '5', capture_age_ms: '161', frame_interval_ms: '52.4',
      thermal_state: 'serious', pose_age_ms: '252',
      relocalization_shift_ft: '1.11',
    }));
  });

  it('rejects malformed JSONL and non-object records with their line number', () => {
    expect(() => missionJsonlToCsv('{"type":"pose"}\n{bad')).toThrow('line 2');
    expect(() => missionJsonlToCsv('[]')).toThrow('line 1');
  });
});
