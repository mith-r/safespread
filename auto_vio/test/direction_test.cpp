#include <cassert>
#include <cstdio>
#include "../direction.h"

static DirectionState readyForward() {
  DirectionState state;
  state.begin(false, false, 0, 0.0f, 0.0f, 0.0f, 1620, 1500);
  assert(state.update(799, 0.0f, 0.0f) == 1500);
  assert(state.phase == D_NEUTRAL);
  assert(state.update(800, 0.0f, 0.0f) == 1620);
  assert(state.phase == D_COMMAND);
  assert(state.update(801, 0.0f, 0.0f) == 1620);
  assert(state.phase == D_VERIFY);
  assert(state.update(900, 0.0f, 0.31f) == 1620);
  assert(state.phase == D_READY && !state.failed());
  return state;
}

static void driveReverseSequence(DirectionState &state) {
  state.begin(true, true, 1000, 2.0f, 4.0f, 0.0f, 1380, 1500);
  assert(state.update(1799, 2.0f, 4.0f) == 1500);
  assert(state.phase == D_NEUTRAL);
  assert(state.update(1800, 2.0f, 4.0f) == 1380);
  assert(state.phase == D_BRAKE);
  assert(state.update(2099, 2.0f, 4.0f) == 1380);
  assert(state.phase == D_BRAKE);
  assert(state.update(2100, 2.0f, 4.0f) == 1500);
  assert(state.phase == D_NEUTRAL);
  assert(state.update(2199, 2.0f, 4.0f) == 1500);
  assert(state.phase == D_NEUTRAL);
  assert(state.update(2200, 2.0f, 4.0f) == 1380);
  assert(state.phase == D_COMMAND);
  assert(state.update(2201, 2.0f, 4.0f) == 1380);
  assert(state.phase == D_VERIFY);
  assert(state.update(2300, 2.0f, 3.69f) == 1380);
  assert(state.phase == D_READY && state.reverse);
}

int main() {
  assert(!driveDirectionChanged(false, false, true));
  assert(!driveDirectionChanged(true, false, false));
  assert(driveDirectionChanged(true, false, true));
  assert(driveDirectionChanged(true, true, false));

  DirectionState navigation = readyForward();

  // A verified reverse change always observes neutral, then the optional
  // brake pulse, before the actual reverse command is judged from motion.
  driveReverseSequence(navigation);

  // Self-test and navigation use the exact same state machine and therefore
  // produce the same phase/output sequence for the same evidence.
  DirectionState selfTest;
  driveReverseSequence(selfTest);
  assert(selfTest.phase == navigation.phase);
  assert(selfTest.outputPulseUs == navigation.outputPulseUs);

  // Reverse -> forward still requires neutral, but never a reverse brake tap.
  navigation.begin(false, true, 3000, 2.0f, 3.69f, 0.0f, 1620, 1500);
  assert(navigation.update(3800, 2.0f, 3.69f) == 1620);
  assert(navigation.phase == D_COMMAND);
  navigation.update(3801, 2.0f, 3.69f);
  navigation.update(3900, 2.0f, 4.01f);
  assert(navigation.phase == D_READY && !navigation.reverse);

  // A command that moves far enough in the opposite sign fails immediately.
  DirectionState wrong;
  wrong.begin(true, false, 0, 0.0f, 0.0f, 0.0f, 1380, 1500);
  wrong.update(800, 0.0f, 0.0f);
  wrong.update(801, 0.0f, 0.0f);
  assert(wrong.update(900, 0.0f, 0.31f) == 1500);
  assert(wrong.phase == D_FAILED && wrong.wrongDirection);

  // No displacement within two seconds of command is a failed verification.
  DirectionState stalled;
  stalled.begin(false, false, 0, 0.0f, 0.0f, 0.0f, 1620, 1500);
  stalled.update(800, 0.0f, 0.0f);
  stalled.update(801, 0.0f, 0.0f);
  assert(stalled.update(2800, 0.0f, 0.0f) == 1500);
  assert(stalled.phase == D_FAILED && stalled.noDisplacement);

  // Heading projection works away from the default +Y field direction.
  DirectionState east;
  east.begin(false, false, 0, 1.0f, 2.0f, 90.0f, 1620, 1500);
  east.update(800, 1.0f, 2.0f);
  east.update(801, 1.0f, 2.0f);
  east.update(900, 1.31f, 2.0f);
  assert(east.phase == D_READY);
  std::printf("direction_test: all assertions passed\n");
  return 0;
}
