#!/usr/bin/env python3
"""为无人值守 PX4 SITL 提供最小、非控制型本地 GCS heartbeat。"""

from __future__ import annotations

import argparse
import signal
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class HeartbeatConfig:
    """本地 MAVLink GCS heartbeat 目标与发送频率。"""

    host: str = "127.0.0.1"
    port: int = 18570
    rate_hz: float = 1.0
    source_system: int = 255
    source_component: int = 190

    def validate(self) -> None:
        """拒绝非法地址参数和过高发送频率。"""

        if not self.host:
            raise ValueError("host must not be empty")
        if self.port <= 0 or self.port > 65535:
            raise ValueError("port must be within [1, 65535]")
        if self.rate_hz <= 0.0 or self.rate_hz > 10.0:
            raise ValueError("rate_hz must be within (0, 10]")
        for label, value in (
            ("source_system", self.source_system),
            ("source_component", self.source_component),
        ):
            if value <= 0 or value > 255:
                raise ValueError(f"{label} must be within [1, 255]")


def heartbeat_fields() -> dict[str, int]:
    """返回不声明控制能力的标准 GCS heartbeat 字段。"""

    return {
        "custom_mode": 0,
        "mav_type": 6,  # MAV_TYPE_GCS
        "autopilot": 8,  # MAV_AUTOPILOT_INVALID
        "base_mode": 0,
        "system_status": 4,  # MAV_STATE_ACTIVE
        "mavlink_version": 3,
    }


def send_heartbeat(connection: Any) -> None:
    """向已建立的 pymavlink 连接发送一帧非控制 heartbeat。"""

    fields = heartbeat_fields()
    connection.mav.heartbeat_send(
        fields["mav_type"],
        fields["autopilot"],
        fields["base_mode"],
        fields["custom_mode"],
        fields["system_status"],
        fields["mavlink_version"],
    )


def run(config: HeartbeatConfig) -> None:
    """持续发送 heartbeat，直到收到 SIGINT/SIGTERM。"""

    config.validate()
    try:
        from pymavlink import mavutil
    except ImportError as error:
        raise RuntimeError(
            "pymavlink is required for unattended PX4 SITL GCS heartbeat"
        ) from error

    connection = mavutil.mavlink_connection(
        f"udpout:{config.host}:{config.port}",
        source_system=config.source_system,
        source_component=config.source_component,
    )
    running = True

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    period_s = 1.0 / config.rate_hz
    next_send = time.monotonic()
    try:
        while running:
            now = time.monotonic()
            if now >= next_send:
                send_heartbeat(connection)
                next_send = now + period_s
            time.sleep(min(0.05, max(0.0, next_send - time.monotonic())))
    finally:
        connection.close()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18570)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--source-system", type=int, default=255)
    parser.add_argument("--source-component", type=int, default=190)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate arguments and print the frozen heartbeat fields without sending",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    config = HeartbeatConfig(
        host=args.host,
        port=args.port,
        rate_hz=args.rate_hz,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    config.validate()
    if args.dry_run:
        print(
            {
                "target": f"udpout:{config.host}:{config.port}",
                "rate_hz": config.rate_hz,
                "source_system": config.source_system,
                "source_component": config.source_component,
                "heartbeat": heartbeat_fields(),
            }
        )
        return 0
    run(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
