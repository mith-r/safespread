# Hardware bring-up sketches

Standalone Arduino sketches used to bring up and calibrate the rover hardware
before the autonomous stack in `auto_vio/` existed. They are not part of the
mission firmware and are not built by `auto_vio/test/run_tests.sh`; each is
opened and flashed on its own from the Arduino IDE.

| Sketch | Written | Purpose |
|---|---|---|
| `safespread_ble_drive` | 2026-08-17 | BLE RC drive test over Bluefruit Connect Control Pad, direct motor pins |
| `safespread_steer_trim` | 2026-08-18 | Steering trim tool — finds the microsecond values for centre and full lock |
| `safespread_ble_pca9685` | 2026-08-23 | BLE RC drive test on the final PCA9685 + Grove relay wiring |

Recovered 2026-09-06 from `Documents/Dartmouth/ENGS 21/Arduino/`, where they had
been sitting outside version control since they were written.
