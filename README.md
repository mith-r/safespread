# SafeSpread

This repository is the canonical source snapshot for the working SafeSpread
iPhone app and rover firmware.

- `SafeSpreadVIO/` contains the Expo/React Native iOS app and native ARKit pose
  module.
- `auto_vio/` contains the Adafruit Metro ESP32-S3 firmware and its host tests.

Generated iOS projects, dependencies, Arduino build products, captured mission
logs, and superseded standalone sketches are intentionally not included.

## Verify

```sh
cd SafeSpreadVIO
npm test -- --runInBand

cd ../auto_vio/test
./run_tests.sh
```

## Build the rover firmware

```sh
arduino-cli compile \
  --fqbn esp32:esp32:adafruit_metro_esp32s3 \
  auto_vio
```
