# Firmware releases

## Version source

The application version is defined once in `include/version.h`. The Home
screen displays that value, and the release packaging script uses it to name
the firmware artifact. Release tags must match it with a `v` prefix; for
example, `CARDTASTIC_VERSION "0.7.0"` is released as `v0.7.0`.

Before creating a release tag:

1. Confirm the changes intended for the release are merged into `main`.
2. Update `include/version.h` and move the relevant `Current` entries in
   `CHANGELOG.md` under the dated version heading.
3. Run the build, native tests, and the agreed hardware checks. Record the
   Cardputer and Meshtastic radio/firmware versions actually tested.
4. Commit the release version and changelog, then create and push the matching
   annotated tag (for example, `v0.7.0`) on that commit.

Pushing a `v*` tag runs `.github/workflows/release.yml`. The workflow verifies
that the tag matches `include/version.h`, builds the firmware, packages it, runs
the native regressions, calculates SHA-256, and publishes the artifacts as a
GitHub Release. No hardware is needed for this automated workflow.

## Reproducing the release package locally

Install the documented PlatformIO Core version and a C++ compiler, then run
from a clean checkout:

```sh
python3 -m pip install platformio==6.1.18
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Isrc test/native/client_state_test.cpp -o /tmp/cardtastic-client-state-test
/tmp/cardtastic-client-state-test
python3 scripts/package_release.py --tag v0.7.0
```

The `--tag` argument is optional for a local build without a release tag. When
provided, it must match the version in `include/version.h`. PlatformIO builds
into `.pio/build/m5stack-cardputer`; the script locates PlatformIO's installed
esptool and `boot_app0.bin` through `pio system info`, rather than assuming a
fixed home-directory path.

## Release artifacts and flashing

The release contains one merged image:

- `cardtastic-v<VERSION>-m5stack-cardputer.bin`
- `SHA256SUMS.txt`

The merged image contains the ESP32-S3 bootloader, partition table, `boot_app0`,
and application firmware at these offsets:

| Component | Flash offset |
| --- | ---: |
| Bootloader | `0x0` |
| Partition table | `0x8000` |
| `boot_app0.bin` | `0xe000` |
| Application firmware | `0x10000` |

Flash the merged image at offset `0x0`. Verify its checksum from the directory
containing the downloaded files:

```sh
sha256sum -c SHA256SUMS.txt
```

## Hardware validation record

Record actual results before describing a release as hardware-validated. Do not
infer radio compatibility from a successful build.

| Cardputer | Meshtastic radio and firmware | Checks performed | Result / date |
| --- | --- | --- | --- |
|  |  |  |  |
