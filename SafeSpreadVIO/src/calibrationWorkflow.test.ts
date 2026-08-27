import {
  appendDiagnosticLine,
  calibrationLineResult,
  calibrationStepName,
  calibrationStepTimeoutMs,
  selfTestLineResult,
} from './calibrationWorkflow';

describe('calibration workflow result handling', () => {
  it('gives loaded motion steps a bounded result window beyond ordinary BLE ACKs', () => {
    expect(calibrationStepName(6)).toMatch(/loaded speed/i);
    expect(calibrationStepTimeoutMs(5)).toBeGreaterThanOrEqual(45000);
    expect(calibrationStepTimeoutMs(6)).toBeGreaterThanOrEqual(45000);
    expect(calibrationStepTimeoutMs(7)).toBeGreaterThanOrEqual(30000);
  });

  it('uses only output-safe step markers as authoritative calibration results', () => {
    expect(calibrationLineResult('[CAL STEP PASS] Motion step finished and outputs are safe.'))
      .toBe('success');
    expect(calibrationLineResult('[CAL STEP FAIL] Motion step stopped; correct the logged cause.'))
      .toBe('failure');
    expect(calibrationLineResult('[CAL SAMPLE] throttle=1700 speed=0.8 ft/s.')).toBeNull();
    expect(calibrationLineResult('[CAL PASS] Motion calibration saved for ID 9.')).toBeNull();
    expect(calibrationLineResult('[CAL FAIL] Speed run stalled.')).toBeNull();
    expect(calibrationLineResult('[CAL SPEED] forward 1/3')).toBeNull();
  });

  it('recognizes only terminal self-test banners', () => {
    expect(selfTestLineResult('=== SELF TEST ===')).toBeNull();
    expect(selfTestLineResult('=== SELF TEST COMPLETE ===')).toBe('success');
    expect(selfTestLineResult('=== SELF TEST FAILED: MOTION NOT VERIFIED ===')).toBe('failure');
  });

  it('keeps a bounded diagnostic transcript so the physical failure remains visible', () => {
    let transcript = '';
    for (let index = 0; index < 12; index += 1) {
      transcript = appendDiagnosticLine(transcript, `line ${index}`, 4);
    }
    expect(transcript).toBe('line 8\nline 9\nline 10\nline 11');
  });
});
