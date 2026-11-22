"""Verified sector-zero upgrade for the current replacement C30D."""
import argparse
import json
import struct
import sys

import flash
from chassis import Chassis

BOARD = ("1a86", "55d4", "5CF0016377")


def read_candidate(version, image_sha, *, archived=False):
    root = flash.ROOT / "Firmware" / version
    path = root / ("bin/firmware.bin" if archived else "src/build/c30d_body.bin")
    image = path.read_bytes()
    if flash.sha(image) != image_sha:
        raise RuntimeError(f"{version} 镜像哈希与验收记录不匹配")
    sp, reset = struct.unpack_from("<II", image)
    if not 0x20000000 < sp <= 0x20020000 or not reset & 1 or not flash.BASE <= reset < flash.BASE + len(image):
        raise RuntimeError("应用向量表无效")
    return image


def install(version, baseline_sector_sha, image_sha, *, archived=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--power-off-confirmed", action="store_true")
    if not parser.parse_args().power_off_confirmed:
        parser.error("需要确认 Motor OFF、物理断开 12V、仅 USB 供电")
    candidate = read_candidate(version, image_sha, archived=archived)
    if archived:
        manifest_path = flash.ROOT / "Firmware" / version / "bin/manifest.json"
        artifact = candidate, json.loads(manifest_path.read_text())
    else:
        artifact = None
    config = flash.load_config()
    _, expected = flash.device(config)
    if expected != BOARD:
        raise RuntimeError("当前底盘身份与替换板不匹配")
    car = Chassis(config)
    try:
        state = car.read_state()
        if state["motor_enabled"] or state["motor_power_present"] or any(state["pwm_ticks"]):
            raise RuntimeError("遥测未确认 Motor OFF、动力断开和四轮 PWM 为零")
    finally:
        car.close()
    print("PRE_FLASH_MOTOR_OFF_POWER_OFF_PWM_ZERO", flush=True)
    (flash.ROOT / "Backups").mkdir(exist_ok=True)
    class VerifiedROM(flash.SectorZeroROM):
        def backup(self, directory, info, **kwargs):
            result = super().backup(directory, info, **kwargs)
            sector = (directory / "flash_read_1.bin").read_bytes()[:flash.SECTOR]
            if flash.sha(sector) != baseline_sector_sha and sector != candidate.ljust(flash.SECTOR, b"\xff"):
                raise RuntimeError(f"板上扇区 0 与已验证基线或 {version} 不匹配，双备份已保存")
            return result

    flash.SectorZeroROM = VerifiedROM
    sys.argv = [sys.argv[0], "--version", version, "--flash", "--power-off-confirmed"]
    flash.main(artifact=artifact)
