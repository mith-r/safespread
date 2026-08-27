export type CalibrationOpcode = 5 | 6 | 7;
export type CalibrationLineResult = 'success' | 'failure' | null;

const STEP_TIMEOUT_MS: Record<CalibrationOpcode, number> = {
  // These are deliberately longer than the current firmware bounds so a
  // loaded breakaway search can evolve without the phone declaring failure
  // while the rover is still safely executing a bounded step.
  5: 60000,
  6: 60000,
  7: 45000,
};

export function calibrationStepName(opcode: CalibrationOpcode): string {
  if (opcode === 5) return 'steering calibration';
  if (opcode === 6) return 'loaded speed calibration';
  return 'forward/reverse verification';
}

export function calibrationStepTimeoutMs(opcode: CalibrationOpcode): number {
  return STEP_TIMEOUT_MS[opcode];
}

export function calibrationLineResult(line: string): CalibrationLineResult {
  const value = line.trim();
  if (value.startsWith('[CAL STEP FAIL]') ||
      value.startsWith('[CAL] A calibration step is already active')) return 'failure';
  if (value.startsWith('[CAL STEP PASS]')) return 'success';
  return null;
}

export function selfTestLineResult(line: string): CalibrationLineResult {
  const value = line.trim();
  if (!value.startsWith('=== SELF TEST')) return null;
  if (value.includes('COMPLETE')) return 'success';
  if (value.includes('FAILED') || value.includes('ABORTED') || value.includes('STOPPED')) {
    return 'failure';
  }
  return null;
}

export function appendDiagnosticLine(
  transcript: string,
  line: string,
  maximumLines = 10,
): string {
  const trimmed = line.trim();
  if (!trimmed) return transcript;
  const previous = transcript.split('\n').filter(Boolean);
  if (previous.at(-1) === trimmed) return transcript;
  const lines = [...previous, trimmed];
  return lines.slice(-Math.max(1, maximumLines)).join('\n');
}
