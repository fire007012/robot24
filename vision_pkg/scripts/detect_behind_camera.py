#!/usr/bin/env python3
"""检测 behind_camera 类型：astra 或 realsense。
   通过 lsusb 查找 USB 设备的 Vendor ID 判断。
   输出 "astra" 或 "realsense"，用于 launch 文件的 arg 默认值。
"""
import subprocess
import sys

# Orbbec (Astra+) Vendor ID
ORBBEC_VID = "2bc5"
# Intel RealSense Vendor ID
REALSENSE_VID = "8086"

# RealSense Product IDs for depth cameras (排除 paw_camera D405)
# D435i = 0x0B3A, D435 = 0x0B07
# D405  = 0x0B5B (要排除，因为它是 paw_camera)
REALSENSE_BEHIND_PIDS = {"0b3a", "0b07", "0b5c", "0b64"}  # D435i, D435, D455, D456
D405_PID = "0b5b"


def detect():
    try:
        output = subprocess.check_output(["lsusb"], text=True)
    except Exception:
        return "unknown"

    has_orbbec = False
    realsense_count = 0
    has_behind_realsense = False

    for line in output.lower().splitlines():
        if ORBBEC_VID in line:
            has_orbbec = True
        if REALSENSE_VID in line:
            realsense_count += 1
            # 检查是否有 D435i 等非 D405 的 RealSense
            for pid in REALSENSE_BEHIND_PIDS:
                if pid in line:
                    has_behind_realsense = True

    if has_orbbec:
        return "astra"
    elif has_behind_realsense:
        return "realsense"
    elif realsense_count >= 2:
        # 有两个以上 RealSense，说明第二个不是 D405
        return "realsense"
    else:
        return "unknown"


if __name__ == "__main__":
    result = detect()
    # 只输出结果，给 launch 文件用
    print(result)
    sys.exit(0)
