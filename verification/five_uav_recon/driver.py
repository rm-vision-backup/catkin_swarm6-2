#!/usr/bin/env python3
"""五机任务下发入口：复用 four_uav_recon/driver.py 的同一份实现。

五机与三/四机的差别只在任务数据（--package）与默认任务号，编排逻辑完全相同，
所以这里不抄第二份 driver，只补默认值后用 os.execv 换成真正的 driver 进程
（退出码、Ctrl-C、信号处理与原命令一致）。

用法（与四机版同参）：
    python3 verification/five_uav_recon/driver.py \
      --tcp-host 192.168.5.241 --tcp-port 39001 \
      --log verification/five_uav_recon/logs/real_$(date +%Y%m%d_%H%M%S).jsonl

不传 --package / --mission-id 时，默认取本目录 package 与 five-uav-recon-001。
"""

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DRIVER = HERE.parent / "four_uav_recon" / "driver.py"

argv = sys.argv[1:]


def _has(flag: str) -> bool:
    return any(item == flag or item.startswith(flag + "=") for item in argv)


if not _has("--package"):
    argv += ["--package", str(HERE / "package")]
if not _has("--mission-id"):
    argv += ["--mission-id", "five-uav-recon-001"]

os.execv(sys.executable, [sys.executable, str(DRIVER)] + argv)
