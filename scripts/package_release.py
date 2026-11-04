#!/usr/bin/env python3
"""Build and package Cardtastic release firmware for the M5Stack Cardputer."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VERSION_HEADER = ROOT / "include" / "version.h"
ENVIRONMENT = "m5stack-cardputer"


def read_version() -> str:
    text = VERSION_HEADER.read_text(encoding="utf-8")
    match = re.search(
        r'^\s*#define\s+CARDTASTIC_VERSION\s+"([^"]+)"\s*$',
        text,
        re.MULTILINE,
    )
    if match is None:
        raise SystemExit(f"Could not read CARDTASTIC_VERSION from {VERSION_HEADER}")

    version = match.group(1)
    if re.fullmatch(r"\d+\.\d+\.\d+(?:-[0-9A-Za-z.-]+)?", version) is None:
        raise SystemExit(f"Invalid semantic version in {VERSION_HEADER}: {version}")
    return version


def find_platformio() -> str:
    executable = shutil.which("pio") or shutil.which("platformio")
    if executable is None:
        raise SystemExit("PlatformIO Core is required; install it and ensure pio is on PATH")
    return executable


def platformio_info(executable: str) -> tuple[Path, Path]:
    result = subprocess.run(
        [executable, "system", "info", "--json-output"],
        check=True,
        capture_output=True,
        text=True,
        cwd=ROOT,
    )
    info = json.loads(result.stdout)
    try:
        core_dir = Path(info["core_dir"]["value"])
        python_exe = Path(info["python_exe"]["value"])
    except (KeyError, TypeError) as exc:
        raise SystemExit("PlatformIO did not report its package and Python directories") from exc
    return core_dir, python_exe


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as artifact:
        for chunk in iter(lambda: artifact.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tag",
        help="release tag to validate (must match v<CARDTASTIC_VERSION>)",
    )
    args = parser.parse_args()

    version = read_version()
    expected_tag = f"v{version}"
    if args.tag is not None and args.tag != expected_tag:
        raise SystemExit(
            f"Tag {args.tag!r} does not match {VERSION_HEADER}: expected {expected_tag!r}"
        )

    pio = find_platformio()
    subprocess.run(
        [pio, "run", "-e", ENVIRONMENT],
        check=True,
        cwd=ROOT,
    )

    core_dir, python_exe = platformio_info(pio)
    build_dir = ROOT / ".pio" / "build" / ENVIRONMENT
    esptool = core_dir / "packages" / "tool-esptoolpy" / "esptool.py"
    boot_app0 = (
        core_dir
        / "packages"
        / "framework-arduinoespressif32"
        / "tools"
        / "partitions"
        / "boot_app0.bin"
    )
    bootloader = build_dir / "bootloader.bin"
    partitions = build_dir / "partitions.bin"
    firmware = build_dir / "firmware.bin"

    required = (esptool, boot_app0, bootloader, partitions, firmware)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("Required release build files are missing:\n" + "\n".join(missing))

    dist = ROOT / "dist"
    dist.mkdir(exist_ok=True)
    artifact = dist / f"cardtastic-{expected_tag}-m5stack-cardputer.bin"
    subprocess.run(
        [
            str(python_exe),
            str(esptool),
            "--chip",
            "esp32s3",
            "merge_bin",
            "-o",
            str(artifact),
            "0x0",
            str(bootloader),
            "0x8000",
            str(partitions),
            "0xe000",
            str(boot_app0),
            "0x10000",
            str(firmware),
        ],
        check=True,
        cwd=ROOT,
    )

    checksum_file = dist / "SHA256SUMS.txt"
    checksum_file.write_text(
        f"{sha256(artifact)}  {artifact.name}\n",
        encoding="utf-8",
    )
    print(f"Release artifact: {artifact}")
    print(f"SHA-256 checksums: {checksum_file}")


if __name__ == "__main__":
    main()
