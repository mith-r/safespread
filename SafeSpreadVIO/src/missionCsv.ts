export const MISSION_CSV_COLUMNS = [
  'phone_ms',
  'sequence',
  'epoch',
  'x_ft',
  'y_ft',
  'heading_deg',
  'speed_fps',
  'yaw_rate_dps',
  'tracking_valid',
  'route_index',
  'cross_track_ft',
  'heading_error_deg',
  'steering_us',
  'throttle_us',
  'fault',
  // Why a pose stream thinned out, and whether the frame it was reported in is
  // the one it was reported in. Reconstructing either from the JSONL by hand is
  // what the last investigation had to do.
  'capture_age_ms',
  'frame_interval_ms',
  'thermal_state',
  'pose_age_ms',
  'relocalization_shift_ft',
] as const;

const RECORD_KEYS: Record<(typeof MISSION_CSV_COLUMNS)[number], string> = {
  phone_ms: 'phoneMs',
  sequence: 'sequence',
  epoch: 'epoch',
  x_ft: 'xFt',
  y_ft: 'yFt',
  heading_deg: 'headingDeg',
  speed_fps: 'speedFps',
  yaw_rate_dps: 'yawRateDps',
  tracking_valid: 'trackingValid',
  route_index: 'routeIndex',
  cross_track_ft: 'crossTrackFt',
  heading_error_deg: 'headingErrorDeg',
  steering_us: 'steeringUs',
  throttle_us: 'throttleUs',
  fault: 'fault',
  capture_age_ms: 'captureAgeMs',
  frame_interval_ms: 'frameIntervalMs',
  thermal_state: 'thermalState',
  pose_age_ms: 'poseAgeMs',
  relocalization_shift_ft: 'relocalizationShiftFt',
};

function csvCell(value: unknown): string {
  if (value === undefined || value === null) return '';
  if (typeof value === 'number' && !Number.isFinite(value)) {
    throw new TypeError('CSV values must be finite');
  }
  const text = typeof value === 'object' ? JSON.stringify(value) : String(value);
  return /[",\r\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
}

function recordValue(
  record: Record<string, unknown>,
  column: (typeof MISSION_CSV_COLUMNS)[number],
): unknown {
  if (column === 'fault') return record.fault ?? record.faultCode;
  return record[RECORD_KEYS[column]];
}

export function missionJsonlToCsv(jsonl: string): string {
  const records: Record<string, unknown>[] = [];
  jsonl.split(/\r?\n/).forEach((line, index) => {
    if (line.trim() === '') return;
    let parsed: unknown;
    try {
      parsed = JSON.parse(line);
    } catch {
      throw new Error(`malformed JSONL at line ${index + 1}`);
    }
    if (parsed === null || Array.isArray(parsed) || typeof parsed !== 'object') {
      throw new Error(`mission JSONL line ${index + 1} is not an object`);
    }
    const record = parsed as Record<string, unknown>;
    if (record.type !== 'metadata') records.push(record);
  });

  const rows = records.map((record) => MISSION_CSV_COLUMNS
    .map((column) => csvCell(recordValue(record, column)))
    .join(','));
  return `${MISSION_CSV_COLUMNS.join(',')}\n${rows.length ? `${rows.join('\n')}\n` : ''}`;
}
