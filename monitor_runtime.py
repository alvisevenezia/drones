#!/usr/bin/env python3
"""Log runtime metrics for the SLAM stack.

Outputs a compact line with:
- RTF computed from /clock vs wall time
- CPU usage from /proc/stat
- RAM usage from /proc/meminfo
- GPU usage from nvidia-smi when available
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


@dataclass
class CpuSample:
    total: int
    idle: int


def read_cpu_sample() -> CpuSample:
    with Path("/proc/stat").open("r", encoding="utf-8") as handle:
        parts = handle.readline().split()
    values = [int(value) for value in parts[1:]]
    idle = values[3] + values[4] if len(values) > 4 else values[3]
    total = sum(values)
    return CpuSample(total=total, idle=idle)


def cpu_percent(previous: Optional[CpuSample], current: CpuSample) -> tuple[Optional[float], CpuSample]:
    if previous is None:
        return None, current
    total_delta = current.total - previous.total
    idle_delta = current.idle - previous.idle
    if total_delta <= 0:
        return None, current
    usage = (1.0 - (idle_delta / total_delta)) * 100.0
    return max(0.0, min(100.0, usage)), current


def read_memory_percent() -> tuple[float, float, float]:
    values: dict[str, float] = {}
    with Path("/proc/meminfo").open("r", encoding="utf-8") as handle:
        for line in handle:
            key, raw_value, *_ = line.split()
            values[key.rstrip(":")] = float(raw_value)
    total_gib = values.get("MemTotal", 0.0) / 1024.0 / 1024.0
    available_gib = values.get("MemAvailable", 0.0) / 1024.0 / 1024.0
    used_gib = max(0.0, total_gib - available_gib)
    percent = (used_gib / total_gib * 100.0) if total_gib > 0.0 else 0.0
    return percent, used_gib, total_gib


def query_gpu() -> str:
    if not shutil.which("nvidia-smi"):
        return "GPU=n/a"

    command = [
        "nvidia-smi",
        "--query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu,power.draw",
        "--format=csv,noheader,nounits",
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        return "GPU=n/a"

    first_line = result.stdout.strip().splitlines()[0] if result.stdout.strip() else ""
    if not first_line:
        return "GPU=n/a"

    parts = [part.strip() for part in first_line.split(",")]
    if len(parts) < 6:
        return "GPU=n/a"

    gpu_util, gpu_mem_util, gpu_mem_used, gpu_mem_total, gpu_temp, gpu_power = parts[:6]
    return (
        f"GPU={gpu_util}% | VRAM={gpu_mem_used}/{gpu_mem_total} MiB ({gpu_mem_util}%) "
        f"| temp={gpu_temp} C | power={gpu_power} W"
    )


class RuntimeMonitor(Node):
    def __init__(self) -> None:
        super().__init__("runtime_monitor")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.last_clock_wall: Optional[float] = None
        self.last_clock_sim: Optional[float] = None
        self.last_rtf: Optional[float] = None
        self.last_cpu: Optional[CpuSample] = None
        self.last_cpu_percent: Optional[float] = None
        self.previous_cpu_sample = read_cpu_sample()
        self.create_timer(1.0, self.report)
        self.get_logger().info("Runtime monitor started: RTF, CPU, RAM, GPU")

    def on_clock(self, msg: Clock) -> None:
        current_wall = time.monotonic()
        current_sim = float(msg.clock.sec) + float(msg.clock.nanosec) / 1_000_000_000.0
        if self.last_clock_wall is not None and self.last_clock_sim is not None:
                        wall_delta = current_wall - self.last_clock_wall
                        sim_delta = current_sim - self.last_clock_sim
                        if wall_delta > 0.0 and sim_delta >= 0.0:
              self.last_rtf = sim_delta / wall_delta
        self.last_clock_wall = current_wall
        self.last_clock_sim = current_sim

    def report(self) -> None:
        cpu, self.previous_cpu_sample = cpu_percent(self.previous_cpu_sample, read_cpu_sample())
        self.last_cpu_percent = cpu if cpu is not None else self.last_cpu_percent
        mem_percent, used_gib, total_gib = read_memory_percent()
        rtf_text = f"{self.last_rtf:.2f}" if self.last_rtf is not None else "n/a"
        cpu_text = f"{self.last_cpu_percent:.1f}%" if self.last_cpu_percent is not None else "n/a"
        gpu_text = query_gpu()
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self.get_logger().info(
            f"{timestamp} | RTF={rtf_text} | CPU={cpu_text} | "
            f"RAM={mem_percent:.1f}% ({used_gib:.1f}/{total_gib:.1f} GiB) | {gpu_text}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Log RTF, CPU, RAM and GPU usage")
    parser.parse_args()

    rclpy.init()
    node = RuntimeMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()