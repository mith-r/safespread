#include <cassert>
#include <cmath>
#include <cstdio>
#include "../speed_control.h"

int main() {
  assert(DEFAULT_STRAIGHT_SPEED_FPS == 1.5f);
  assert(DEFAULT_TURN_SPEED_FPS == 0.7f);

  SpeedPI atSpeed;
  atSpeed.feedForwardUs = 120.0f;
  assert(atSpeed.update(1.0f, 1.0f, 0.05f) == 120);
  assert(std::fabs(atSpeed.integralUs) < 0.001f);

  // Reverse uses the same learned magnitude with a signed target/output.
  SpeedPI reverse;
  reverse.feedForwardUs = 125.0f;
  assert(reverse.update(-1.0f, -1.0f, 0.05f) == -125);

  // PI correction is bounded independently, then the complete ESC offset is
  // clamped to the absolute 350 us hardware envelope.
  SpeedPI bounded;
  bounded.feedForwardUs = 200.0f;
  assert(bounded.update(10.0f, 0.1f, 0.05f) == 400);
  assert(std::fabs(bounded.lastCorrectionUs) <= 300.0f);

  // Saturation does not wind the integrator farther into either rail.
  SpeedPI upper;
  upper.feedForwardUs = 300.0f;
  upper.integralUs = 100.0f;
  upper.update(1.0f, 0.5f, 0.5f);
  assert(std::fabs(upper.integralUs - 100.0f) < 0.001f);

  SpeedPI lower;
  lower.feedForwardUs = 300.0f;
  lower.integralUs = -100.0f;
  lower.update(-1.0f, -0.5f, 0.5f);
  assert(std::fabs(lower.integralUs + 100.0f) < 0.001f);

  // With no measured motion, use a bounded breakaway ramp rather than
  // integrating an ever-growing throttle request.
  SpeedPI breakaway;
  breakaway.feedForwardUs = 120.0f;
  int first = breakaway.update(1.0f, 0.0f, 0.2f);
  int second = breakaway.update(1.0f, 0.0f, 0.2f);
  assert(first > 120 && second > first);
  assert(std::fabs(breakaway.integralUs) < 0.001f);

  // Full breakaway is reached after one second, then gets two seconds to work.
  // This prevents the controller from faulting at the exact instant it first
  // supplies maximum starting power on wet ground.
  SpeedPI noMotion;
  noMotion.feedForwardUs = 120.0f;
  for (int index = 0; index < 14; ++index) noMotion.update(1.0f, 0.0f, 0.2f);
  assert(!noMotion.stalled);
  assert(std::fabs(noMotion.breakawayUs - SpeedPI::MAX_BREAKAWAY_US) < 0.001f);
  assert(noMotion.lastOffsetUs > 0);
  noMotion.update(1.0f, 0.0f, 0.2f);
  assert(noMotion.stalled);
  assert(noMotion.lastOffsetUs == 0);
  const float frozenIntegral = noMotion.integralUs;
  assert(noMotion.update(1.0f, 0.0f, 0.2f) == 0);
  assert(noMotion.integralUs == frozenIntegral);
  noMotion.reset();
  assert(!noMotion.stalled && noMotion.lastOffsetUs == 0);

  // Invalid timing samples are ignored without modifying controller state.
  SpeedPI badDt;
  badDt.feedForwardUs = 120.0f;
  int prior = badDt.update(1.0f, 0.8f, 0.1f);
  const float priorIntegral = badDt.integralUs;
  assert(badDt.update(1.0f, 0.8f, 0.0f) == prior);
  assert(badDt.update(1.0f, 0.8f, 1.0f) == prior);
  assert(std::isnan(0.0f / 0.0f));
  assert(badDt.update(1.0f, 0.8f, 0.0f / 0.0f) == prior);
  assert(badDt.integralUs == priorIntegral);

  // A rover the loop has to actually drive, plus the delay it is seen through.
  // Steady-state speed is how far the pulse sits past the ESC deadband divided
  // by the microseconds this load needs per foot per second; the lag stands in
  // for the mass. The two loads bracket what 3ce5c37 measured on pavement --
  // +190us light and +370us loaded held 1.8 ft/s, so 67 and 167 us per ft/s
  // once the 70us deadband comes off.
  struct Rover {
    float usPerFps;
    float deadbandUs = 70.0f;
    float tauSeconds = 0.4f;
    float speedFps = 0.0f;

    void step(int offsetUs, float dt) {
      const float past = std::fabs((float)offsetUs) - deadbandUs;
      float settled = past <= 0.0f ? 0.0f : past / usPerFps;
      if (offsetUs < 0) settled = -settled;
      speedFps += (settled - speedFps) * (dt / tauSeconds);
    }
  };

  // The controller never sees the current speed. The phone medians five poses
  // before it sends one and a pose may be 250ms old, so feed it a delayed
  // sample -- without this the gain could be raised arbitrarily here and the
  // rover would ring on pavement.
  struct Lag {
    float samples[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float push(float value) {
      const float oldest = samples[0];
      for (int i = 0; i < 3; ++i) samples[i] = samples[i + 1];
      samples[3] = value;
      return oldest;
    }
  };

  const float DT = 0.05f;   // 20 Hz, about what BLE delivers poses at
  const float LIGHT_US_PER_FPS = 67.0f;
  const float LOADED_US_PER_FPS = 167.0f;

  struct Run { float peakFps; float settledSeconds; };
  auto drive = [&](SpeedPI &loop, Rover &rover, Lag &lag, float target,
                   float seconds) {
    Run out = {0.0f, -1.0f};
    const int steps = (int)(seconds / DT);
    for (int i = 0; i < steps; ++i) {
      rover.step(loop.update(target, lag.push(rover.speedFps), DT), DT);
      if (rover.speedFps > out.peakFps) out.peakFps = rover.speedFps;
      if (out.settledSeconds < 0.0f &&
          std::fabs(rover.speedFps - target) < 0.05f) {
        out.settledSeconds = i * DT;
      }
    }
    return out;
  };

  // A full tank has to reach cruise well inside a pass. At 1.5 ft/s a 14 ft
  // pass lasts about nine seconds.
  SpeedPI loaded;
  loaded.feedForwardUs = 120.0f;
  Rover loadedRover{LOADED_US_PER_FPS};
  Lag loadedLag;
  Run loadedRun = drive(loaded, loadedRover, loadedLag,
                        DEFAULT_STRAIGHT_SPEED_FPS, 9.0f);
  std::printf("speed_control_test: loaded rover reaches %.2f ft/s in %.1fs"
              " on +%dus\n",
              loadedRover.speedFps, loadedRun.settledSeconds,
              loaded.lastOffsetUs);
  assert(loadedRun.settledSeconds >= 0.0f && loadedRun.settledSeconds < 6.0f);
  assert(std::fabs(loadedRover.speedFps - DEFAULT_STRAIGHT_SPEED_FPS) < 0.05f);
  assert(!loaded.stalled);

  // The reason the loop exists: the tank drains over a mission, the rover gets
  // lighter, and a held throttle would run it faster and faster. Drain it the
  // way it really empties -- gradually -- and the loop has to give the throttle
  // back the whole way without the speed ever leaving the band.
  const int fullTankOffsetUs = loaded.lastOffsetUs;
  float worstDeviationFps = 0.0f;
  const int drainSteps = (int)(60.0f / DT);
  for (int i = 0; i < drainSteps; ++i) {
    const float emptied = (float)i / (float)drainSteps;
    loadedRover.usPerFps =
        LOADED_US_PER_FPS + (LIGHT_US_PER_FPS - LOADED_US_PER_FPS) * emptied;
    loadedRover.step(
        loaded.update(DEFAULT_STRAIGHT_SPEED_FPS,
                      loadedLag.push(loadedRover.speedFps), DT), DT);
    const float deviation =
        std::fabs(loadedRover.speedFps - DEFAULT_STRAIGHT_SPEED_FPS);
    if (deviation > worstDeviationFps) worstDeviationFps = deviation;
  }
  std::printf("speed_control_test: tank drains full->empty, worst error"
              " %.3f ft/s, throttle +%dus -> +%dus\n",
              worstDeviationFps, fullTankOffsetUs, loaded.lastOffsetUs);
  assert(worstDeviationFps < 0.05f);
  assert(loaded.lastOffsetUs < fullTankOffsetUs - 100);
  assert(!loaded.stalled);

  // An empty rover is where the loop is closest to ringing, so hold it to a
  // small overshoot. Raising the gain until this trips is the failure this
  // guards: the rover would surge past cruise at the head of every pass.
  SpeedPI light;
  light.feedForwardUs = 120.0f;
  Rover lightRover{LIGHT_US_PER_FPS};
  Lag lightLag;
  Run lightRun = drive(light, lightRover, lightLag,
                       DEFAULT_STRAIGHT_SPEED_FPS, 9.0f);
  std::printf("speed_control_test: empty rover reaches %.2f ft/s in %.1fs"
              " on +%dus, peak %.2f\n",
              lightRover.speedFps, lightRun.settledSeconds,
              light.lastOffsetUs, lightRun.peakFps);
  assert(lightRun.settledSeconds >= 0.0f && lightRun.settledSeconds < 2.5f);
  assert(lightRun.peakFps < DEFAULT_STRAIGHT_SPEED_FPS + 0.06f);

  // Loading a rover that was holding cruise light: it has to find the extra
  // throttle rather than just running slow.
  const int lightOffsetUs = light.lastOffsetUs;
  lightRover.usPerFps = LOADED_US_PER_FPS;
  drive(light, lightRover, lightLag, DEFAULT_STRAIGHT_SPEED_FPS, 12.0f);
  assert(std::fabs(lightRover.speedFps - DEFAULT_STRAIGHT_SPEED_FPS) < 0.05f);
  assert(light.lastOffsetUs > lightOffsetUs + 50);

  // A rover that cannot reach the target pushes up against the ceiling and
  // stays there. It parks just under it rather than exactly on it, because the
  // anti-windup stops feeding the integrator the moment another microsecond
  // would take the offset past the limit -- which is the point.
  SpeedPI pinned;
  pinned.feedForwardUs = 120.0f;
  Rover heavyRover{600.0f};
  Lag heavyLag;
  drive(pinned, heavyRover, heavyLag, DEFAULT_STRAIGHT_SPEED_FPS, 8.0f);
  const int pinnedOffsetUs = pinned.lastOffsetUs;
  const float pinnedIntegralUs = pinned.integralUs;
  assert(pinnedOffsetUs <= (int)SpeedPI::MAX_OFFSET_US);
  assert(pinnedOffsetUs > (int)SpeedPI::MAX_OFFSET_US - 15);
  assert(std::fabs(pinned.lastCorrectionUs) <= SpeedPI::MAX_CORRECTION_US);
  drive(pinned, heavyRover, heavyLag, DEFAULT_STRAIGHT_SPEED_FPS, 8.0f);
  assert(pinned.lastOffsetUs == pinnedOffsetUs);
  assert(pinned.integralUs == pinnedIntegralUs);

  // The first target after a reset is taken whole. Ramping up from a standstill
  // would soften the push that breaks the rover away, which is the one thing
  // the stall timeout is watching for.
  SpeedPI ramp;
  ramp.feedForwardUs = 120.0f;
  ramp.update(DEFAULT_STRAIGHT_SPEED_FPS, 1.0f, DT);
  assert(ramp.rampedTargetFps == DEFAULT_STRAIGHT_SPEED_FPS);

  // Entering a turn the target walks down instead of stepping, so the
  // integrator unwinds over the approach rather than into the turn.
  ramp.update(DEFAULT_TURN_SPEED_FPS, 1.4f, 0.1f);
  assert(std::fabs(ramp.rampedTargetFps -
                   (DEFAULT_STRAIGHT_SPEED_FPS -
                    SpeedPI::MAX_TARGET_SLEW_FPS2 * 0.1f)) < 0.001f);
  for (int i = 0; i < 20; ++i) ramp.update(DEFAULT_TURN_SPEED_FPS, 0.9f, 0.1f);
  assert(ramp.rampedTargetFps == DEFAULT_TURN_SPEED_FPS);

  // A direction change is a discontinuity, not a slew: never ramp through zero.
  ramp.update(-DEFAULT_TURN_SPEED_FPS, -0.1f, DT);
  assert(ramp.rampedTargetFps == -DEFAULT_TURN_SPEED_FPS);

  // Neutral is immediate at any point in a ramp. A stop is never slewed.
  assert(ramp.update(0.0f, -0.7f, DT) == 0);
  assert(ramp.rampedTargetFps == 0.0f);
  assert(!ramp.targetInitialized);

  ramp.update(DEFAULT_STRAIGHT_SPEED_FPS, 1.0f, DT);
  ramp.reset();
  assert(ramp.rampedTargetFps == 0.0f && !ramp.targetInitialized);

  std::printf("speed_control_test: all assertions passed\n");
  return 0;
}
