Import("env")
import os

def merge_bin(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    esptool = os.path.join(env.PioPlatform().get_package_dir("tool-esptool"), "esptool.exe")
    if not os.path.exists(esptool):
        esptool = os.path.join(env.PioPlatform().get_package_dir("tool-esptool"), "esptool.py")
    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    firmware = os.path.join(build_dir, "firmware.bin")
    merged = os.path.join(build_dir, "firmware_merged.bin")
    cmd = (
        f'"{esptool}" --chip esp32 merge-bin -o "{merged}" '
        f'--flash-mode dio --flash-freq 40m --flash-size 4MB '
        f'0x1000 "{bootloader}" 0x8000 "{partitions}" 0x10000 "{firmware}"'
    )
    env.Execute(cmd)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_bin)

